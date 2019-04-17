cmake_minimum_required(VERSION 2.8.12)
project(imgui)

add_library(imgui STATIC
    imgui.cpp
    imgui_impl_glfw.cpp
    imgui_draw.cpp
    )

set_target_properties(imgui PROPERTIES OUTPUT_NAME "imgui")
target_include_directories(imgui PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(OpenGL REQUIRED)

find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    target_include_directories(imgui PUBLIC ${GLEW_INCLUDE_DIRS})
    target_link_libraries(imgui ${GLEW_LIBRARIES} ${OPENGL_LIBRARY})
endif()

#if (GLFW3_FOUND)
#    target_include_directories(imgui PUBLIC ${GLFW3_INCLUDE_DIRS})
#    target_link_libraries(imgui ${GLFW3_LIBRARIES} ${OPENGL_LIBRARY})
#endif ()

if(APPLE)
    find_library(COCOA_LIBRARY Cocoa)
    find_library(OPENGL_LIBRARY OpenGL)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(COREVIDEO_LIBRARY CoreVideo)
    set(EXTRA_LIBS
        ${COCOA_LIBRARY}
        ${OPENGL_LIBRARY}
        ${IOKIT_LIBRARY}
        ${COREVIDEO_LIBRARY}
        -lglfw3
#        gif
        )
    target_link_libraries(imgui ${EXTRA_LIBS})
endif(APPLE)

install(TARGETS "imgui" ARCHIVE DESTINATION lib)
