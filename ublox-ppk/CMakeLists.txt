cmake_minimum_required(VERSION 3.16)
project(ublox_ppk LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_subdirectory(logger)   # ubx_logger : no dependencies
add_subdirectory(ppk)      # ppk_solve  : fetches + builds RTKLIB demo5
