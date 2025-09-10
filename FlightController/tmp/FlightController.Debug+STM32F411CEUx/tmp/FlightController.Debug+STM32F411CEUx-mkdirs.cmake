# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/FlightController.Debug+STM32F411CEUx")
  file(MAKE_DIRECTORY "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/FlightController.Debug+STM32F411CEUx")
endif()
file(MAKE_DIRECTORY
  "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/1"
  "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/FlightController.Debug+STM32F411CEUx"
  "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/FlightController.Debug+STM32F411CEUx/tmp"
  "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/FlightController.Debug+STM32F411CEUx/src/FlightController.Debug+STM32F411CEUx-stamp"
  "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/FlightController.Debug+STM32F411CEUx/src"
  "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/FlightController.Debug+STM32F411CEUx/src/FlightController.Debug+STM32F411CEUx-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/FlightController.Debug+STM32F411CEUx/src/FlightController.Debug+STM32F411CEUx-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/colin-tormanen/Documents/Fall 2025/ECE 461/Source Code/FlightController/tmp/FlightController.Debug+STM32F411CEUx/src/FlightController.Debug+STM32F411CEUx-stamp${cfgdir}") # cfgdir has leading slash
endif()
