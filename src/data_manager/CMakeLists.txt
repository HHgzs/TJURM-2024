file(GLOB_RECURSE
    data_src 
        ${CMAKE_SOURCE_DIR}/src/data_manager/*.cpp
)

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/json json_binary_dir)
set(JSON_LIBS 
        nlohmann_json::nlohmann_json
        -lgxiapi
        -lpthread
)


add_library(data_manager
    STATIC
        ${data_src}    
)

target_link_libraries(data_manager
    PUBLIC
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
        ${JSON_LIBS}
)
