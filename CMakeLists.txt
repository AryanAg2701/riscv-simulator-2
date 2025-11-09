cmake_minimum_required(VERSION 3.21)

project(RiscVSimulator LANGUAGES CXX)

# Use GCC 15 explicitly (full path to avoid CMake cache confusion)
set(CMAKE_C_COMPILER "/opt/homebrew/Cellar/gcc/15.1.0/bin/gcc-15" CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_COMPILER "/opt/homebrew/Cellar/gcc/15.1.0/bin/g++-15" CACHE FILEPATH "" FORCE)

# Force C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ENABLE_TESTS "Build tests" OFF)

set(SRC_DIR "src")
set(INCLUDE_DIR "include")
set(TEST_DIR "test")

file(GLOB_RECURSE SRC_FILES "${SRC_DIR}/*.cpp")
file(GLOB_RECURSE TEST_FILES "${TEST_DIR}/*.cpp")

add_executable(vm ${SRC_FILES})
target_include_directories(vm PRIVATE ${INCLUDE_DIR})
target_compile_options(vm PRIVATE -Wall -Wextra -pedantic -frounding-math -ffloat-store -g -O3)

if(ENABLE_ASAN)
    message(STATUS "ASAN enabled: Adding AddressSanitizer flags")
    target_compile_options(vm PRIVATE -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)
    target_link_options(vm PRIVATE -fsanitize=address)
endif()

if(ENABLE_TESTS)
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})
    list(REMOVE_ITEM SRC_FILES "${CMAKE_SOURCE_DIR}/src/main.cpp")
    add_executable(tests ${SRC_FILES} ${TEST_FILES})
    target_link_libraries(tests GTest::GTest GTest::Main pthread)
    add_custom_target(test_run
        COMMAND ./tests
        DEPENDS tests
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endif()

add_custom_target(run
    COMMAND ./vm
    DEPENDS vm
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_custom_target(debug
    COMMAND gdb ./vm
    DEPENDS vm
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_custom_target(valgrind
    COMMAND valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes ./vm
    DEPENDS vm
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Doxygen docs
find_package(Doxygen)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating documentation with Doxygen"
    )
endif()

# macOS specific: enable filesystem linking
# Filesystem handling (portable)
if(APPLE)
    add_definitions(-D_LIBCPP_ENABLE_CXX17_REMOVED_FEATURES)
    # Modern libc++ and libstdc++ no longer require -lc++fs
    message(STATUS "✅ macOS detected: no extra filesystem link required.")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # For older GCC (pre-9), filesystem required explicit linking.
    include(CheckCXXSymbolExists)
    check_cxx_symbol_exists("std::filesystem::exists" "filesystem" HAS_FS)
    if (NOT HAS_FS)
        find_library(STDCXXFS_LIB stdc++fs)
        if(STDCXXFS_LIB)
            target_link_libraries(vm PRIVATE ${STDCXXFS_LIB})
            message(STATUS "Linked stdc++fs for older GCC filesystem support.")
        endif()
    endif()
endif()


message(STATUS "✅ Configuration complete: using GCC 15 and C++20")
