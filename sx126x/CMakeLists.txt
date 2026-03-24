find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD libgpiod>=2.0)

if(GPIOD_FOUND)
    target_sources(${PROJECT_NAME} PRIVATE gpio_hal_v2.c)
else()
    pkg_check_modules(GPIOD REQUIRED libgpiod)
    target_sources(${PROJECT_NAME} PRIVATE gpio_hal_v1.c)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${GPIOD_LIBRARIES})

target_sources(${PROJECT_NAME} PUBLIC
    sx126x.c
)
