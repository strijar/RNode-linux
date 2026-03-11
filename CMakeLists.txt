cmake_minimum_required(VERSION 3.16)

project(rnode)

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_DEBUG "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O3")

add_executable(${PROJECT_NAME})

add_subdirectory(src)
add_subdirectory(sx126x)

include_directories(src)
include_directories(sx126x)

target_link_libraries(${PROJECT_NAME} PRIVATE m cyaml)

install(FILES rnode.yaml DESTINATION /mnt/rns)
install(TARGETS ${PROJECT_NAME} DESTINATION sbin)
