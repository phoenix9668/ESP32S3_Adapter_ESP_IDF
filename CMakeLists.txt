# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

file(READ "${CMAKE_CURRENT_LIST_DIR}/version.txt" RELEASE_VERSION_FILE)
string(STRIP "${RELEASE_VERSION_FILE}" RELEASE_VERSION_FILE)
if(DEFINED ENV{APP_RELEASE_VERSION} AND NOT "$ENV{APP_RELEASE_VERSION}" STREQUAL "")
    set(RELEASE_VERSION "$ENV{APP_RELEASE_VERSION}")
else()
    set(RELEASE_VERSION "${RELEASE_VERSION_FILE}")
endif()
if(NOT RELEASE_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+([.-][0-9A-Za-z]+)*$")
    message(FATAL_ERROR "Invalid application version: ${RELEASE_VERSION}")
endif()
string(LENGTH "${RELEASE_VERSION}" RELEASE_VERSION_LENGTH)
if(RELEASE_VERSION_LENGTH GREATER 20)
    message(FATAL_ERROR "Application version must be at most 20 characters for OneNET OTA")
endif()
set(PROJECT_VER "${RELEASE_VERSION}")

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
message(STATUS "Application release version: ${PROJECT_VER}")
