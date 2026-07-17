#include "at_parser.h"

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
