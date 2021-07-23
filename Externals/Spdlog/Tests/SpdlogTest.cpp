/** @file SpdLogTests.cpp
 * Created at 2020-03-05
 * Copyright (c) RVBUST, Inc - All rights reserved.
 */

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

int main(int argc, char *argv[]) {
    auto logger = spdlog::stdout_color_mt("console");
    // Loggers can store in a ring buffer all messages (including debug/trace) and display later on
    // demand. When needed, call dump_backtrace() to see them
    logger->enable_backtrace(32);  // create ring buffer with capacity of 32  messages
    // or my_logger->enable_backtrace(32)..
    for (int i = 0; i < 10000; i++) {
        // TODO: call following line will also output all log messages, this is should be bug

        // logger->log(static_cast<spdlog::level::level_enum>(3), "AAAA");

        // But use following line is fine.
        logger->debug("Backtrace message {}", i);  // not logged yet..
    }
    // e.g. if some error happened:
    logger->dump_backtrace();  // log them now! show the last 32 messages

    logger->disable_backtrace();
    return 0;
}
