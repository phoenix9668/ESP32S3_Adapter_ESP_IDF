#include "at_parser.h"

#include <cerrno>
#include <climits>
#include <cstdlib>

size_t at_find_unquoted_comma(const std::string &values, size_t start) {
    bool in_quotes = false;
    for (size_t cursor = start; cursor < values.size(); ++cursor) {
        if (values[cursor] == '"') {
            in_quotes = !in_quotes;
        } else if (values[cursor] == ',' && !in_quotes) {
            return cursor;
        }
    }
    return std::string::npos;
}

AtMhttpFrameResult at_extract_mhttp_content_frame(const std::string &buffer,
                                                  std::string &values,
                                                  size_t &consumed,
                                                  AtMhttpFrameError *error) {
    static const std::string prefix = "+MHTTPURC: ";
    static const std::string content_type = "\"content\"";
    values.clear();
    consumed = 0U;
    if (error != nullptr) {
        *error = AtMhttpFrameError{};
    }

    const auto malformed = [&](size_t search_start, size_t invalid_offset,
                               unsigned char invalid_byte,
                               size_t encoded_received,
                               size_t encoded_expected) {
        const size_t next_frame = buffer.find(prefix, search_start);
        consumed = next_frame == std::string::npos ? buffer.size() : next_frame;
        if (error != nullptr) {
            error->encoded_received = encoded_received;
            error->encoded_expected = encoded_expected;
            error->invalid_offset = invalid_offset;
            error->invalid_byte = invalid_byte;
        }
        values.clear();
        return AtMhttpFrameResult::Malformed;
    };

    if (buffer.compare(0U, prefix.size(), prefix) != 0) {
        return AtMhttpFrameResult::NotContent;
    }
    const size_t values_start = prefix.size();
    const size_t first_comma = at_find_unquoted_comma(buffer, values_start);
    if (first_comma == std::string::npos) {
        return AtMhttpFrameResult::NeedMore;
    }
    if (buffer.substr(values_start, first_comma - values_start) !=
        content_type) {
        return AtMhttpFrameResult::NotContent;
    }

    size_t comma = first_comma;
    size_t previous_comma = comma;
    // Find the comma following httpid, content_len, sum_len and cur_len.
    for (unsigned field = 0U; field < 4U; ++field) {
        previous_comma = comma;
        comma = at_find_unquoted_comma(buffer, comma + 1U);
        if (comma == std::string::npos) {
            return AtMhttpFrameResult::NeedMore;
        }
    }

    const std::string current_length_text =
        buffer.substr(previous_comma + 1U, comma - previous_comma - 1U);
    if (current_length_text.empty()) {
        return malformed(comma + 1U, previous_comma + 1U, 0U, 0U, 0U);
    }
    char *end = nullptr;
    errno = 0;
    const unsigned long current_length =
        std::strtoul(current_length_text.c_str(), &end, 10);
    if (errno != 0 || end != current_length_text.c_str() +
                                current_length_text.size() ||
        current_length > (SIZE_MAX / 2U)) {
        return malformed(comma + 1U, previous_comma + 1U, 0U, 0U, 0U);
    }

    const size_t encoded_length = static_cast<size_t>(current_length) * 2U;
    const auto is_hex = [](unsigned char ch) {
        return (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F') ||
               (ch >= 'a' && ch <= 'f');
    };

    values.assign(buffer, values_start, comma + 1U - values_start);
    values.reserve(values.size() + encoded_length);

    // ML307C may wrap the HEX body itself with CRLF, not only place one CRLF
    // between metadata and data. Count HEX digits instead of raw UART bytes so
    // a 1460-byte body remains one logical URC regardless of line wrapping.
    size_t cursor = comma + 1U;
    size_t encoded = 0U;
    while (encoded < encoded_length) {
        if (cursor >= buffer.size()) {
            values.clear();
            return AtMhttpFrameResult::NeedMore;
        }
        const unsigned char ch = static_cast<unsigned char>(buffer[cursor]);
        if (is_hex(ch)) {
            values.push_back(static_cast<char>(ch));
            ++encoded;
            ++cursor;
            continue;
        }
        // ML307 normally wraps with CRLF, but long sustained transfers have
        // also produced a lone CR or LF at a 510-byte transport boundary.
        // Neither can represent a HEX nibble, so accepting them independently
        // is unambiguous and still leaves the image MD5 as the final guard.
        if (ch == '\r' || ch == '\n') {
            ++cursor;
            continue;
        }
        // Search from the invalid byte itself. If it is the '+' beginning the
        // next URC, searching at cursor + 1 would skip the only resync point.
        return malformed(cursor, cursor, ch, encoded, encoded_length);
    }

    consumed = cursor;
    while (consumed < buffer.size() &&
           (buffer[consumed] == '\r' || buffer[consumed] == '\n')) {
        ++consumed;
    }
    return AtMhttpFrameResult::Complete;
}
