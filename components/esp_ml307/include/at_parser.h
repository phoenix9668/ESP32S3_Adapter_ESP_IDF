#ifndef AT_PARSER_H
#define AT_PARSER_H

#include <cstddef>
#include <string>

// Returns the next comma outside a double-quoted AT field, or npos.
size_t at_find_unquoted_comma(const std::string &values, size_t start);

enum class AtMhttpFrameResult {
    NotContent,
    NeedMore,
    Complete,
    Malformed,
};

// ML307 may insert CRLF between the +MHTTPURC content metadata and its HEX
// payload, and does not provide a reliable line suffix after the payload.
// Extract one complete content frame based on <cur_len> instead of CRLF.
AtMhttpFrameResult at_extract_mhttp_content_frame(const std::string &buffer,
                                                  std::string &values,
                                                  size_t &consumed);

#endif
