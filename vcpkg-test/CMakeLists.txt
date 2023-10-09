cmake_minimum_required(VERSION 3.18)

set(VCPKG_FEATURE_FLAGS "versions")
project(vcpkg-test CXX)

add_executable(main src/main.cpp)

find_package(ZLIB REQUIRED)
find_package(fmt CONFIG REQUIRED)
target_link_libraries(main PRIVATE fmt::fmt-header-only ZLIB::ZLIB)