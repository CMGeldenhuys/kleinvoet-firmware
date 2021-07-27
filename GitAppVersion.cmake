# Code taken from: https://cmake.org/pipermail/cmake/2018-October/068389.html
# Author: Matt Schulte
# Date: 11 Oct. 2018

find_package(Git QUIET REQUIRED)

execute_process(
        COMMAND "${GIT_EXECUTABLE}" describe --abbrev=4 --dirty=* --always --tags
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE res
        OUTPUT_VARIABLE APP_VERSION
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE)

set_property(GLOBAL APPEND
        PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${CMAKE_SOURCE_DIR}/.git/index")

string(REGEX REPLACE "^([0-9]+)\\.([0-9]+)\\.([0-9]+).*$"
        "\\1;\\2;\\3" _ver_parts "${APP_VERSION}")
list(GET _ver_parts 0 APP_VERSION_MAJOR)
list(GET _ver_parts 1 APP_VERSION_MINOR)
list(GET _ver_parts 2 APP_VERSION_PATCH)

if("${APP_VERSION}" MATCHES "^.*-(.*)-g.*$")
    string(REGEX REPLACE "^.*-(.*)-g.*$" "\\1" APP_VERSION_MICRO
            "${APP_VERSION}")
else()
    set(APP_VERSION_MICRO "0")
endif()
configure_file(${INPUT_FILE} ${OUTPUT_FILE})
message(STATUS "APP Version: ${APP_VERSION}")