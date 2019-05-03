cmake_minimum_required(VERSION 3.1.0)

project(RSLogger)
#add_subdirectory(thirdparty/glfw)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(OpenCV REQUIRED)
find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED) ${GLUT_LIBRARY}
find_package(glfw3)

# imgui
add_subdirectory(thirdparty/imgui)
include_directories(thirdparty/imgui)
link_libraries(imgui)

add_executable(rs-logger rs-logger.cpp raw.cpp)
target_link_libraries(rs-logger  ${OPENGL_LIBRARIES}  glfw ${realsense_INCLUDE_DIR} ${REALSENSE2_FOUND}
          ${OpenCV_LIBS})
include_directories(rs-logger  ${realsense_INCLUDE_DIR} thirdparty/imgui)
#set_target_properties (rs-logger PROPERTIES FOLDER Examples)
#install(TARGETS rs-logger RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
