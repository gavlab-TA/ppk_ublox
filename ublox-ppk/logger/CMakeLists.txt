add_executable(ubx_logger src/ubx_logger.cpp)
target_compile_features(ubx_logger PRIVATE cxx_std_17)
target_compile_options(ubx_logger PRIVATE -Wall -Wextra)
install(TARGETS ubx_logger RUNTIME DESTINATION bin)
