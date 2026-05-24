cmake_minimum_required(VERSION 3.10)

project(converter LANGUAGES C)

add_library(utf8-utf16-converter src/converter.c)

target_include_directories(utf8-utf16-converter PUBLIC include)
