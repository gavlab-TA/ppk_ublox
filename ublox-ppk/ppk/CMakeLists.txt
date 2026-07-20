include(FetchContent)

# --- Fetch demo5 RTKLIB (rtklibexplorer fork) ---------------------------------
# demo5 is substantially better than official 2.4.3 for multi-GNSS / F9P PPK.
FetchContent_Declare(
  rtklib
  GIT_REPOSITORY https://github.com/rtklibexplorer/RTKLIB.git
  GIT_TAG        demo5
  GIT_SHALLOW    TRUE
)

# We only want the sources populated; we build our own static lib target so we
# control the compile defines (constellations) and don't build RTKLIB's apps.
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)   # allow FetchContent_Populate on CMake >= 3.30
endif()
FetchContent_GetProperties(rtklib)
if(NOT rtklib_POPULATED)
  FetchContent_Populate(rtklib)
endif()

file(GLOB RTKLIB_CORE CONFIGURE_DEPENDS "${rtklib_SOURCE_DIR}/src/*.c")
file(GLOB RTKLIB_RCV  CONFIGURE_DEPENDS "${rtklib_SOURCE_DIR}/src/rcv/*.c")
# comnav.c and tersus.c are orphaned in demo5: their STRFMT_ ids are commented
# out in rtklib.h and nothing in the dispatch references them, yet they retain
# stale code that no longer compiles. Neither is needed for u-blox/RINEX PPK.
list(FILTER RTKLIB_RCV EXCLUDE REGEX "(comnav|tersus)\\.c$")

add_library(rtklib STATIC ${RTKLIB_CORE} ${RTKLIB_RCV})
target_include_directories(rtklib PUBLIC "${rtklib_SOURCE_DIR}/src")
target_compile_definitions(rtklib PUBLIC
  ENAGLO ENAGAL ENACMP ENAQZS TRACE)          # enable GLO/GAL/BDS/QZSS
target_compile_options(rtklib PRIVATE
  -w -O2 -fcommon                             # RTKLIB is warning-noisy; -fcommon for gcc10+
  -fsigned-char)                              # ARM/RPi: char is unsigned by default; RTKLIB assumes signed
set_target_properties(rtklib PROPERTIES C_STANDARD 99)

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)     # --base-dir accepts gzipped RINEX straight from CORS

# --- our solver ---------------------------------------------------------------
add_executable(ppk_solve src/ppk_solve.cpp)
target_compile_features(ppk_solve PRIVATE cxx_std_17)
target_link_libraries(ppk_solve PRIVATE rtklib Threads::Threads ZLIB::ZLIB m)

# ship the default config next to the binary for convenience
add_custom_command(TARGET ppk_solve POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          "${CMAKE_CURRENT_SOURCE_DIR}/config/f9p_kinematic.conf"
          "$<TARGET_FILE_DIR:ppk_solve>/f9p_kinematic.conf")

install(TARGETS ppk_solve RUNTIME DESTINATION bin)
install(FILES config/f9p_kinematic.conf DESTINATION share/ublox_ppk)
