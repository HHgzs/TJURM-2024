file(GLOB_RECURSE
    garage_src
        ${CMAKE_SOURCE_DIR}/src/garage/*.cpp      
)

add_library(garage
    STATIC
        ${garage_src}
)

target_link_libraries(garage
    PUBLIC
        ${OpenRM_LIBS}
        data_manager
)