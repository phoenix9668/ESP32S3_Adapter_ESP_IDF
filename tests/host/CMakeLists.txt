cmake_minimum_required(VERSION 3.16)
project(esp32_adapter_host_tests C CXX)

enable_testing()
set(IDF_CJSON_DIR
    "/Users/gally/.espressif/v5.5.4/esp-idf/components/json/cJSON")
set(IDF_MBEDTLS_DIR
    "/Users/gally/.espressif/v5.5.4/esp-idf/components/mbedtls/mbedtls")
set(ENABLE_PROGRAMS OFF CACHE BOOL "" FORCE)
set(ENABLE_TESTING OFF CACHE BOOL "" FORCE)
add_subdirectory(${IDF_MBEDTLS_DIR} ${CMAKE_BINARY_DIR}/mbedtls EXCLUDE_FROM_ALL)
add_executable(protocol_tests
    test_protocols.c
    ../../main/app_protocol.c
    ../../main/app_protocol_stream.c
    ../../main/gnss_ml307c.c
    ../../main/onenet_ota_protocol.c
    ../../main/onenet_reply.c
    ../../main/onenet_time.c
    ../../main/onenet_token.c
    ../../main/rfid_response.c
    ${IDF_CJSON_DIR}/cJSON.c
)
target_include_directories(protocol_tests PRIVATE
    stubs
    ../../main
    ${IDF_CJSON_DIR}
)
target_link_libraries(protocol_tests m mbedcrypto)
add_test(NAME protocol_tests COMMAND protocol_tests)

add_executable(rfid_store_tests
    test_rfid_store.c
    ../../main/app_protocol.c
    ../../main/rfid_store.c
)
target_compile_definitions(rfid_store_tests PRIVATE RFID_STORE_HOST_TEST)
target_include_directories(rfid_store_tests PRIVATE stubs ../../main)
add_test(NAME rfid_store_tests COMMAND rfid_store_tests)

add_executable(at_parser_tests
    test_at_parser.cc
    ../../components/esp_ml307/src/at_parser.cc
)
target_include_directories(at_parser_tests PRIVATE
    ../../components/esp_ml307/include
)
add_test(NAME at_parser_tests COMMAND at_parser_tests)
