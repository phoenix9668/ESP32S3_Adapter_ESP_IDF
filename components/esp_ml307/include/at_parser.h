#ifndef AT_PARSER_H
#define AT_PARSER_H

#include <cstddef>
#include <string>

// Returns the next comma outside a double-quoted AT field, or npos.
size_t at_find_unquoted_comma(const std::string &values, size_t start);

#endif
