# Build PC de opossum_control (tests, simulation SIL). Le firmware Zynq compile
# directement src/ dans le projet Vitis opossum_core2 (pas ce fichier).
cmake_minimum_required(VERSION 3.10)
project(opossum_control C)

option(OPOSSUM_CONTROL_TESTS "Compiler et enregistrer les tests" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

file(GLOB CONTROL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
set(CONTROL_PC_SOURCES ${CONTROL_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/pc/control_hal_pc.c)

# Bibliotheque statique (tests, binaires C) et partagee (ctypes / pybind pour la SIL)
add_library(opossum_control STATIC ${CONTROL_PC_SOURCES})
add_library(opossum_control_shared SHARED ${CONTROL_PC_SOURCES})
set_target_properties(opossum_control_shared PROPERTIES OUTPUT_NAME opossum_control)

foreach(t opossum_control opossum_control_shared)
  target_include_directories(${t} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/pc)
  target_compile_options(${t} PRIVATE -Wall)
  target_link_libraries(${t} PUBLIC m)
  set_target_properties(${t} PROPERTIES C_STANDARD 11 POSITION_INDEPENDENT_CODE ON)
endforeach()

if(OPOSSUM_CONTROL_TESTS)
  enable_testing()
  add_executable(test_control pc/test_control.c)
  target_link_libraries(test_control opossum_control)
  target_compile_options(test_control PRIVATE -Wall -Wextra)
  add_test(NAME test_control COMMAND test_control)
endif()
