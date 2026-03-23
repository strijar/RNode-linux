cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_C_FLAGS_DEBUG "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O3")

project(rnode)

add_executable(${PROJECT_NAME})

find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE gpiod m cyaml Threads::Threads)

add_subdirectory(src)
add_subdirectory(sx126x)

include_directories(src)
include_directories(sx126x)

install(FILES rnode.yaml DESTINATION /mnt/rns)
install(TARGETS ${PROJECT_NAME} DESTINATION sbin)
