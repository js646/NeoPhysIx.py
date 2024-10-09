cmake_minimum_required(VERSION 3.27)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler settings (Adjust based on your compiler)
set(CMAKE_C_COMPILER "C:/MinGW/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "C:/MinGW/bin/g++.exe")

# Tells the compiler to link the DLLs statically, meaning directly inside the .pyd binary file
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static")

# Set path to python exe
set(Python_EXECUTABLE "C:/Users/Julian/AppData/Local/Programs/Python/Python311/python.exe")

# Set pybind11_DIR path
set(pybind11_DIR "C:/Users/Julian/AppData/Local/Programs/Python/Python311/Lib/site-packages/pybind11/share/cmake/pybind11")

project(NeoPhysix)

find_package(pybind11 CONFIG REQUIRED)
include_directories(${pybind11_INCLUDE_DIRS})
message([MAIN] "Found pybind11 v${pybind11_VERSION}: ${pybind11_INCLUDE_DIRS}")

add_library(NeoPhysix SHARED
    src/PhysicsEngine.cpp
    src/PhysicsAPI.cpp
    src/Environment.cpp
    src/RigidBody.cpp
    src/Math.cpp
    src/pyMain.cpp
    pybind_module.cpp
)

target_include_directories(NeoPhysix PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(NeoPhysix PRIVATE pybind11::module)
