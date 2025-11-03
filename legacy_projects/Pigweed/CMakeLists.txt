cmake_minimum_required(VERSION 3.13)

# Set project name and the C++ standard
project(MySTM32Project LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

# Include Pigweed build system
include(pigweed/repo/pw_build/pigweed.cmake)

# Set the backend for pw_assert
pw_set_backend(pw_assert.assert, pw_assert.basic)

# Set the path to your STM32CubeMX generated code
set(CUBEMX_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# Include STM32 HAL and CMSIS headers
include_directories(
  ${CUBEMX_PATH}/Core/Inc
  ${CUBEMX_PATH}/Drivers/STM32F4xx_HAL_Driver/Inc
  ${CUBEMX_PATH}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
  ${CUBEMX_PATH}/Drivers/CMSIS/Include
)

# Add source files
file(GLOB_RECURSE SOURCES
  ${CUBEMX_PATH}/Core/Src/*.c
  ${CUBEMX_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/*.c
)

# Include Pigweed headers
include_directories(
  pigweed
  pigweed/pw_rpc
  pigweed/pw_rpc_transport
  pigweed/pw_protobuf
)

# Add executable
add_executable(MySTM32Project ${SOURCES})

# Link Pigweed libraries
target_link_libraries(MySTM32Project
  pw_rpc
  pw_rpc_transport
  pw_protobuf
)

# Link STM32 HAL libraries
target_link_libraries(MySTM32Project
  stm32f4xx_hal
  cmsis
)
