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

    const std::string wrapped_content =
        "+MHTTPURC: \"content\",0,724992,1460,6,\r\n"
        "E901\r\n02AF\r\na055\r\n";
    assert(at_extract_mhttp_content_frame(wrapped_content, values, consumed) ==
           AtMhttpFrameResult::Complete);
    assert(values == "\"content\",0,724992,1460,6,E90102AFa055");
    assert(consumed == wrapped_content.size());

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
    const std::string partial_wrapped =
        "+MHTTPURC: \"content\",0,724992,1085,4,E901\r";
    assert(at_extract_mhttp_content_frame(partial_wrapped, values, consumed) ==
           AtMhttpFrameResult::NeedMore);

    const std::string lone_separators =
        "+MHTTPURC: \"content\",0,724992,1460,6,\r"
        "E901\n02AF\ra055\n";
    assert(at_extract_mhttp_content_frame(lone_separators, values, consumed) ==
           AtMhttpFrameResult::Complete);
    assert(values == "\"content\",0,724992,1460,6,E90102AFa055");
    assert(consumed == lone_separators.size());

    const std::string malformed_then_valid =
        "+MHTTPURC: \"content\",0,724992,1085,4,E901ZZZZ"
        "+MHTTPURC: \"content\",0,724992,1089,4,0011AAFF";
    assert(at_extract_mhttp_content_frame(malformed_then_valid, values,
                                          consumed) ==
           AtMhttpFrameResult::Malformed);
    assert(consumed == malformed_then_valid.find("+MHTTPURC", 1U));

    AtMhttpFrameError error;
    const std::string truncated_then_valid =
        "+MHTTPURC: \"content\",0,724992,1085,4,E901"
        "+MHTTPURC: \"content\",0,724992,1089,4,0011AAFF";
    assert(at_extract_mhttp_content_frame(truncated_then_valid, values,
                                          consumed, &error) ==
           AtMhttpFrameResult::Malformed);
    assert(consumed == truncated_then_valid.find("+MHTTPURC", 1U));
    assert(error.encoded_received == 4U);
    assert(error.encoded_expected == 8U);
    assert(error.invalid_byte == static_cast<unsigned char>('+'));
    assert(error.invalid_offset == consumed);
    assert(at_extract_mhttp_content_frame("+CPIN: READY\r\n", values,
                                          consumed) ==
           AtMhttpFrameResult::NotContent);
    return 0;
}
