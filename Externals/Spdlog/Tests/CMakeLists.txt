add_executable(SpdlogTest SpdlogTest.cpp)
target_link_libraries(SpdlogTest PRIVATE spdlog::spdlog)

add_test(NAME SpdlogTest COMMAND SpdlogTest)
