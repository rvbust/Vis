find_package(Threads REQUIRED)

# build shared option
if(NOT WIN32)
    option(SPDLOG_BUILD_SHARED "Build shared library" OFF)
endif()

option(SPDLOG_FMT_EXTERNAL "Use external fmt library instead of bundled" ON)
option(SPDLOG_FMT_EXTERNAL_HO "Use external fmt header-only library instead of bundled" OFF)
option(SPDLOG_NO_EXCEPTIONS "Compile with -fno-exceptions. Call abort() on any spdlog exceptions" ON)

if (SPDLOG_FMT_EXTERNAL AND SPDLOG_FMT_EXTERNAL_HO)
    message(FATAL_ERROR "SPDLOG_FMT_EXTERNAL and SPDLOG_FMT_EXTERNAL_HO are mutually exclusive")
endif()

# misc tweakme options
if(WIN32)
    option(SPDLOG_WCHAR_SUPPORT "Support wchar api" OFF)
    option(SPDLOG_WCHAR_FILENAMES "Support wchar filenames" OFF)
endif()
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    option(SPDLOG_CLOCK_COARSE "Use the much faster (but much less accurate) CLOCK_REALTIME_COARSE instead of the regular clock," OFF)
endif()

option(SPDLOG_PREVENT_CHILD_FD "Prevent from child processes to inherit log file descriptors" OFF)
option(SPDLOG_NO_THREAD_ID "prevent spdlog from querying the thread id on each log call if thread id is not needed" OFF)
option(SPDLOG_NO_TLS "prevent spdlog from using thread local storage" OFF)
option(SPDLOG_NO_ATOMIC_LEVELS "prevent spdlog from using of std::atomic log levels (use only if your code never modifies log levels concurrently" OFF)

find_package(Threads REQUIRED)
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
#---------------------------------------------------------------------------------------
# Static/Shared library (shared not supported in windows yet)
#---------------------------------------------------------------------------------------
set(SPDLOG_SRCS
        src/spdlog.cpp
        src/stdout_sinks.cpp
        src/color_sinks.cpp
        src/file_sinks.cpp
        src/async.cpp)

if(NOT SPDLOG_FMT_EXTERNAL AND NOT SPDLOG_FMT_EXTERNAL_HO)
    list(APPEND SPDLOG_SRCS src/fmt.cpp)
endif()

if (SPDLOG_BUILD_SHARED)
    if(WIN32)
        message(FATAL_ERROR "spdlog shared lib is not yet supported under windows")
    endif()
    add_library(spdlog SHARED ${SPDLOG_SRCS} ${SPDLOG_ALL_HEADERS})
else()
    add_library(spdlog STATIC ${SPDLOG_SRCS} ${SPDLOG_ALL_HEADERS})
endif()

add_library(spdlog::spdlog ALIAS spdlog)

target_compile_definitions(spdlog PUBLIC SPDLOG_COMPILED_LIB)
target_include_directories(spdlog PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
      target_link_libraries(spdlog PUBLIC Threads::Threads)

# #---------------------------------------------------------------------------------------
# # Header only version
# #---------------------------------------------------------------------------------------
# add_library(spdlog_header_only INTERFACE)
# add_library(spdlog::spdlog_header_only ALIAS spdlog_header_only)

# target_include_directories(spdlog_header_only INTERFACE
#         "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
#         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
# target_link_libraries(spdlog_header_only INTERFACE Threads::Threads)


#---------------------------------------------------------------------------------------
# Use fmt package if using external fmt
#---------------------------------------------------------------------------------------
if(SPDLOG_FMT_EXTERNAL OR SPDLOG_FMT_EXTERNAL_HO)
    if (NOT TARGET fmt::fmt)
        find_package(fmt REQUIRED)
    endif ()
    target_compile_definitions(spdlog PUBLIC SPDLOG_FMT_EXTERNAL)
    # target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_FMT_EXTERNAL)

    # use external fmt-header-nly
    if(SPDLOG_FMT_EXTERNAL_HO)
        target_link_libraries(spdlog PUBLIC fmt::fmt-header-only)
        # target_link_libraries(spdlog_header_only INTERFACE fmt::fmt-header-only)
    else() # use external compile fmt
        target_link_libraries(spdlog PUBLIC fmt::fmt)
        # target_link_libraries(spdlog_header_only INTERFACE fmt::fmt)
    endif()

    set(PKG_CONFIG_REQUIRES fmt) # add dependency to pkg-config
endif()

#---------------------------------------------------------------------------------------
# Misc definitions according to tweak options
#---------------------------------------------------------------------------------------
if(SPDLOG_WCHAR_SUPPORT)
	target_compile_definitions(spdlog PUBLIC SPDLOG_WCHAR_TO_UTF8_SUPPORT)
	# target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_WCHAR_TO_UTF8_SUPPORT)
 endif()

 if(SPDLOG_WCHAR_FILENAMES)
	target_compile_definitions(spdlog PUBLIC SPDLOG_WCHAR_FILENAMES)
	# target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_WCHAR_FILENAMES)
 endif()

 if(SPDLOG_NO_EXCEPTIONS)
	target_compile_definitions(spdlog PUBLIC SPDLOG_NO_EXCEPTIONS)	

	# target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_NO_EXCEPTIONS)

    if(NOT MSVC)
        target_compile_options(spdlog PRIVATE -fno-exceptions)
    endif()
endif()

if(SPDLOG_CLOCK_COARSE)
    target_compile_definitions(spdlog PRIVATE SPDLOG_CLOCK_COARSE)
    # target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_CLOCK_COARSE)
endif()

if(SPDLOG_PREVENT_CHILD_FD)
    target_compile_definitions(spdlog PRIVATE SPDLOG_PREVENT_CHILD_FD)
    # target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_PREVENT_CHILD_FD)
endif()

if(SPDLOG_NO_THREAD_ID)
    target_compile_definitions(spdlog PRIVATE SPDLOG_NO_THREAD_ID)
    # target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_NO_THREAD_ID)
endif()

if(SPDLOG_NO_TLS)
    target_compile_definitions(spdlog PRIVATE SPDLOG_NO_TLS)
    # target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_NO_TLS)
endif()

if(SPDLOG_NO_ATOMIC_LEVELS)
    target_compile_definitions(spdlog PUBLIC SPDLOG_NO_ATOMIC_LEVELS)
    # target_compile_definitions(spdlog_header_only INTERFACE SPDLOG_NO_ATOMIC_LEVELS)
endif()

if (RVX_BUILD_TESTS)
  add_subdirectory(Tests)
endif()
