cmake_minimum_required(VERSION 3.16)
project(UnitTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(INC ${CMAKE_CURRENT_SOURCE_DIR}/../../inc)

function(add_unit_test TEST_NAME)
    add_executable(${TEST_NAME} ${TEST_NAME}.c ${ARGN})
    target_compile_definitions(${TEST_NAME} PRIVATE UNIT_TEST)
    target_include_directories(${TEST_NAME} PRIVATE ${SRC} ${INC})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

# Tests 
add_unit_test(test_crsf ${SRC}/crsf.c)
add_unit_test(test_optical_flow_parse ${SRC}/optical_flow.c)
