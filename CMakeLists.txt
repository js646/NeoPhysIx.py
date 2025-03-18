cmake_minimum_required(VERSION 3.15)

# Project name and C++ standard
project(NeoPhysIx LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Python and Pybind11
find_package(Python REQUIRED COMPONENTS Interpreter Development)

# Note: If pybind11 is not found automatically, set the path explicitly.
# Example: set(pybind11_DIR "path/to/pybind11/share/cmake/pybind11")
# e.g: set(pybind11_DIR "/usr/local/Cellar/pybind11/2.13.6_1/share/cmake/pybind11")
# e.g: set(pybind11_DIR "C:/Users/username/AppData/Local/Programs/Python/Python311/Lib/site-packages/pybind11/share/cmake/pybind11")

find_package(pybind11 REQUIRED)

# Debug information
message(STATUS "Found Python: ${Python_EXECUTABLE} (version: ${Python_VERSION})")
message(STATUS "Found pybind11: ${pybind11_VERSION}")

# Add library source
add_library(NeoPhysIx SHARED
    src/PhysicsEngine.cpp
    src/Environment.cpp
    src/RigidBody.cpp
    src/Math.cpp
    src/Control.cpp
    pybind_module.cpp
)

# platform-specific settings. builds ".pyd" on Windows and ".so" on mac and unix
if(WIN32)
    set_target_properties(NeoPhysIx PROPERTIES PREFIX "" SUFFIX ".pyd")
elseif(APPLE OR UNIX)
    set_target_properties(NeoPhysIx PROPERTIES PREFIX "" SUFFIX ".so")
endif()

# Add include directories
target_include_directories(NeoPhysIx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${pybind11_INCLUDE_DIRS})

# Link library with pybind11
target_link_libraries(NeoPhysIx PRIVATE pybind11::module)

# Platform-specific compiler flags
if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static")
elseif(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
elseif(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

