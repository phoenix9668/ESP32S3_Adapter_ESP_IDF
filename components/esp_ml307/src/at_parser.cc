#include "at_parser.h"

#include <algorithm>
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
                                                  size_t &consumed) {
    static const std::string prefix = "+MHTTPURC: ";
    static const std::string content_type = "\"content\"";
    values.clear();
    consumed = 0U;

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
        return AtMhttpFrameResult::Malformed;
    }
    char *end = nullptr;
    errno = 0;
    const unsigned long current_length =
        std::strtoul(current_length_text.c_str(), &end, 10);
    if (errno != 0 || end != current_length_text.c_str() +
                                current_length_text.size() ||
        current_length > (SIZE_MAX / 2U)) {
        return AtMhttpFrameResult::Malformed;
    }

    const size_t encoded_length = static_cast<size_t>(current_length) * 2U;
    size_t payload_start = comma + 1U;
    if (payload_start < buffer.size() && buffer[payload_start] == '\r') {
        if (payload_start + 1U >= buffer.size()) {
            return AtMhttpFrameResult::NeedMore;
        }
        if (buffer[payload_start + 1U] != '\n') {
            return AtMhttpFrameResult::Malformed;
        }
        payload_start += 2U;
    }
    if (buffer.size() - payload_start < encoded_length) {
        return AtMhttpFrameResult::NeedMore;
    }

    const size_t payload_end = payload_start + encoded_length;
    const auto is_hex = [](unsigned char ch) {
        return (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F') ||
               (ch >= 'a' && ch <= 'f');
    };
    if (!std::all_of(buffer.begin() + payload_start,
                     buffer.begin() + payload_end, is_hex)) {
        return AtMhttpFrameResult::Malformed;
    }

    values.assign(buffer, values_start, comma + 1U - values_start);
    values.append(buffer, payload_start, encoded_length);
    consumed = payload_end;
    if (buffer.size() >= consumed + 2U && buffer[consumed] == '\r' &&
        buffer[consumed + 1U] == '\n') {
        consumed += 2U;
    }
    return AtMhttpFrameResult::Complete;
}
