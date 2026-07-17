# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(REQUIRED_IDF_VERSION "5.5.4")
set(ACTUAL_IDF_VERSION
    "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")
if(NOT ACTUAL_IDF_VERSION STREQUAL REQUIRED_IDF_VERSION)
    message(FATAL_ERROR
        "This project requires ESP-IDF v${REQUIRED_IDF_VERSION}; "
        "configured with v${ACTUAL_IDF_VERSION} from $ENV{IDF_PATH}")
endif()

project(ESP32S3_Adapter_ESP_IDF)
