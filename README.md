# NeoPhysIx.py
This application is a Python wrapper for the NeoPhysIx C++ library, allowing NeoPhysIx functionalities to be used in Python. The application uses PyOpenGL for 3D rendering and Pygame for visualization and interaction.

## Installing Python Dependencies
To run the application, the following Python libraries need to be installed. You can install them easily with pip:

```bash
pip install pyopengl pygame numpy
```

## Running the Application
To start the application, execute the main.py file in the python directory:

python python/main.py

## Rebuilding the NeoPhysIx Library
If changes are made to the NeoPhysIx C++ codebase or the pybind11 bindings, the library needs to be rebuilt to apply these changes in the Python environment.

The following dependencies must be installed to rebuild the library:
```bash
pip install pybind11 cmake
```
Make sure the paths in the CMakeLists.txt file are configured correctly.

To compile the library, follow these steps:

Navigate to the build directory (you might have to create it first):

```bash
cd build
```

Run the following commands to create the library:

Windows:
```bash
cmake -G "MinGW Makefiles" ..
cmake --build .
```
MacOS/Linux:
```bash
cmake -G "Unix Makefiles" ..
cmake --build .
```

Move the generated .pyd/.so file to the python directory:

```bash
ren libNeoPhysIx.dll NeoPhysIx.pyd
move NeoPhysIx.pyd ..\python\
```

The updated library is now ready for use in the Python application.
