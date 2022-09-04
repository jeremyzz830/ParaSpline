add_executable(test_1 test.cpp)

target_include_directories(test_1 PUBLIC ${PROJECT_SOURCE_DIR}/lib)

target_link_libraries(test_1 PUBLIC ${PROJECT_BINARY_DIR}/lib/libheader.a)