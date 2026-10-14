cmake_minimum_required(VERSION 3.16)
project(sdr_dsp_host_tests C)

set(CMAKE_C_STANDARD 99)
add_compile_options(-Wall -Wextra -Werror)

add_subdirectory(../../components/sdr_dsp sdr_dsp)

enable_testing()
add_executable(test_sdr_dsp test_sdr_dsp.c)
target_link_libraries(test_sdr_dsp PRIVATE sdr_dsp)
add_test(NAME sdr_dsp COMMAND test_sdr_dsp)
