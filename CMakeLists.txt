cmake_minimum_required(VERSION 3.16)

project(rnode)

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_DEBUG "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O3")

find_package(PkgConfig REQUIRED)

add_executable(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PRIVATE ${GPIOD_INCLUDE_DIRS})

add_subdirectory(src)
add_subdirectory(sx126x)

include_directories(src)
include_directories(sx126x)

target_link_libraries(${PROJECT_NAME} PRIVATE ${GPIOD_LIBRARIES} m cyaml)

install(FILES rnode.yaml DESTINATION /mnt/rns)
install(TARGETS ${PROJECT_NAME} DESTINATION sbin)
