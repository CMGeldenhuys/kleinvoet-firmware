# Code taken from: https://cmake.org/pipermail/cmake/2018-October/068389.html
# Author: Matt Schulte
# Date: 11 Oct. 2018

find_package(Git QUIET REQUIRED)

execute_process(
        COMMAND "${GIT_EXECUTABLE}" describe --abbrev=4 --dirty=* --always --tags
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE res
        OUTPUT_VARIABLE MYAPP_VERSION
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE)

set_property(GLOBAL APPEND
        PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${CMAKE_SOURCE_DIR}/.git/index")

string(REGEX REPLACE "^([0-9]+)\\.([0-9]+)\\.([0-9]+).*$"
        "\\1;\\2;\\3" _ver_parts "${MYAPP_VERSION}")
list(GET _ver_parts 0 MYAPP_VERSION_MAJOR)
list(GET _ver_parts 1 MYAPP_VERSION_MINOR)
list(GET _ver_parts 2 MYAPP_VERSION_PATCH)

if("${MYAPP_VERSION}" MATCHES "^.*-(.*)-g.*$")
    string(REGEX REPLACE "^.*-(.*)-g.*$" "\\1" MYAPP_VERSION_MICRO
            "${MYAPP_VERSION}")
else()
    set(MYAPP_VERSION_MICRO "0")
endif()

message(STATUS "APP Version: ${MYAPP_VERSION}")