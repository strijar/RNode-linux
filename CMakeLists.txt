cmake_minimum_required(VERSION 3.16)

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_C_FLAGS_DEBUG "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O3")

project(rnode)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod>=2.0)

add_executable(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PRIVATE gpiod m cyaml)

add_subdirectory(src)
add_subdirectory(sx126x)

include_directories(src)
include_directories(sx126x)

install(FILES rnode.yaml DESTINATION /mnt/rns)
install(TARGETS ${PROJECT_NAME} DESTINATION sbin)
