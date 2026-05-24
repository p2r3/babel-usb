FILE(GLOB_RECURSE app_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
)

set(utf_converter_src
    "${CMAKE_SOURCE_DIR}/lib/utf8-utf16-converter/converter/src/converter.c"
)

set(utf_converter_inc
    "${CMAKE_SOURCE_DIR}/lib/utf8-utf16-converter/converter/include"
)

idf_component_register(
    SRCS ${app_sources} ${utf_converter_src}
    INCLUDE_DIRS ${utf_converter_inc}
    REQUIRES espressif-tinyusb-component
)
