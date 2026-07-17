#include "at_parser.h"

#include <cassert>
#include <string>

int main() {
    const std::string clock = "\"26/07/17,12:34:56+32\"";
    assert(at_find_unquoted_comma(clock, 0U) == std::string::npos);

    const std::string fields = "\"clock,with,commas\",,42,S";
    const size_t first = at_find_unquoted_comma(fields, 0U);
    assert(first == 19U);
    const size_t empty = at_find_unquoted_comma(fields, first + 1U);
    assert(empty == first + 1U);
    const size_t numeric = at_find_unquoted_comma(fields, empty + 1U);
    assert(fields.substr(empty + 1U, numeric - empty - 1U) == "42");
    assert(at_find_unquoted_comma(fields, numeric + 1U) == std::string::npos);
    return 0;
}
