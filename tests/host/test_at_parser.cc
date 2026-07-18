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

    std::string values;
    size_t consumed = 0U;
    const std::string split_content =
        "+MHTTPURC: \"content\",0,724992,1085,4,\r\nE90102AF\r\n";
    assert(at_extract_mhttp_content_frame(split_content, values, consumed) ==
           AtMhttpFrameResult::Complete);
    assert(values == "\"content\",0,724992,1085,4,E90102AF");
    assert(consumed == split_content.size());

    const std::string inline_content =
        "+MHTTPURC: \"content\",0,724992,1089,4,0011aAff";
    assert(at_extract_mhttp_content_frame(inline_content, values, consumed) ==
           AtMhttpFrameResult::Complete);
    assert(values == "\"content\",0,724992,1089,4,0011aAff");
    assert(consumed == inline_content.size());

    const std::string partial =
        "+MHTTPURC: \"content\",0,724992,1085,4,\r\nE901";
    assert(at_extract_mhttp_content_frame(partial, values, consumed) ==
           AtMhttpFrameResult::NeedMore);
    assert(at_extract_mhttp_content_frame("+CPIN: READY\r\n", values,
                                          consumed) ==
           AtMhttpFrameResult::NotContent);
    return 0;
}
