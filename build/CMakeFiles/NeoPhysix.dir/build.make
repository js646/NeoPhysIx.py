# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.29

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\Julian\Desktop\neophysix.py

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\Julian\Desktop\neophysix.py\build

# Include any dependencies generated for this target.
include CMakeFiles/NeoPhysix.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/NeoPhysix.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/NeoPhysix.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/NeoPhysix.dir/flags.make

CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.obj: CMakeFiles/NeoPhysix.dir/flags.make
CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.obj: CMakeFiles/NeoPhysix.dir/includes_CXX.rsp
CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.obj: C:/Users/Julian/Desktop/neophysix.py/src/PhysicsEngine.cpp
CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.obj: CMakeFiles/NeoPhysix.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\Julian\Desktop\neophysix.py\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.obj"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.obj -MF CMakeFiles\NeoPhysix.dir\src\PhysicsEngine.cpp.obj.d -o CMakeFiles\NeoPhysix.dir\src\PhysicsEngine.cpp.obj -c C:\Users\Julian\Desktop\neophysix.py\src\PhysicsEngine.cpp

CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.i"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Julian\Desktop\neophysix.py\src\PhysicsEngine.cpp > CMakeFiles\NeoPhysix.dir\src\PhysicsEngine.cpp.i

CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.s"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Julian\Desktop\neophysix.py\src\PhysicsEngine.cpp -o CMakeFiles\NeoPhysix.dir\src\PhysicsEngine.cpp.s

CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.obj: CMakeFiles/NeoPhysix.dir/flags.make
CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.obj: CMakeFiles/NeoPhysix.dir/includes_CXX.rsp
CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.obj: C:/Users/Julian/Desktop/neophysix.py/src/PhysicsAPI.cpp
CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.obj: CMakeFiles/NeoPhysix.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\Julian\Desktop\neophysix.py\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.obj"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.obj -MF CMakeFiles\NeoPhysix.dir\src\PhysicsAPI.cpp.obj.d -o CMakeFiles\NeoPhysix.dir\src\PhysicsAPI.cpp.obj -c C:\Users\Julian\Desktop\neophysix.py\src\PhysicsAPI.cpp

CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.i"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Julian\Desktop\neophysix.py\src\PhysicsAPI.cpp > CMakeFiles\NeoPhysix.dir\src\PhysicsAPI.cpp.i

CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.s"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Julian\Desktop\neophysix.py\src\PhysicsAPI.cpp -o CMakeFiles\NeoPhysix.dir\src\PhysicsAPI.cpp.s

CMakeFiles/NeoPhysix.dir/src/Environment.cpp.obj: CMakeFiles/NeoPhysix.dir/flags.make
CMakeFiles/NeoPhysix.dir/src/Environment.cpp.obj: CMakeFiles/NeoPhysix.dir/includes_CXX.rsp
CMakeFiles/NeoPhysix.dir/src/Environment.cpp.obj: C:/Users/Julian/Desktop/neophysix.py/src/Environment.cpp
CMakeFiles/NeoPhysix.dir/src/Environment.cpp.obj: CMakeFiles/NeoPhysix.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\Julian\Desktop\neophysix.py\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/NeoPhysix.dir/src/Environment.cpp.obj"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/NeoPhysix.dir/src/Environment.cpp.obj -MF CMakeFiles\NeoPhysix.dir\src\Environment.cpp.obj.d -o CMakeFiles\NeoPhysix.dir\src\Environment.cpp.obj -c C:\Users\Julian\Desktop\neophysix.py\src\Environment.cpp

CMakeFiles/NeoPhysix.dir/src/Environment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/NeoPhysix.dir/src/Environment.cpp.i"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Julian\Desktop\neophysix.py\src\Environment.cpp > CMakeFiles\NeoPhysix.dir\src\Environment.cpp.i

CMakeFiles/NeoPhysix.dir/src/Environment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/NeoPhysix.dir/src/Environment.cpp.s"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Julian\Desktop\neophysix.py\src\Environment.cpp -o CMakeFiles\NeoPhysix.dir\src\Environment.cpp.s

CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.obj: CMakeFiles/NeoPhysix.dir/flags.make
CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.obj: CMakeFiles/NeoPhysix.dir/includes_CXX.rsp
CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.obj: C:/Users/Julian/Desktop/neophysix.py/src/RigidBody.cpp
CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.obj: CMakeFiles/NeoPhysix.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\Julian\Desktop\neophysix.py\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.obj"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.obj -MF CMakeFiles\NeoPhysix.dir\src\RigidBody.cpp.obj.d -o CMakeFiles\NeoPhysix.dir\src\RigidBody.cpp.obj -c C:\Users\Julian\Desktop\neophysix.py\src\RigidBody.cpp

CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.i"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Julian\Desktop\neophysix.py\src\RigidBody.cpp > CMakeFiles\NeoPhysix.dir\src\RigidBody.cpp.i

CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.s"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Julian\Desktop\neophysix.py\src\RigidBody.cpp -o CMakeFiles\NeoPhysix.dir\src\RigidBody.cpp.s

CMakeFiles/NeoPhysix.dir/src/Math.cpp.obj: CMakeFiles/NeoPhysix.dir/flags.make
CMakeFiles/NeoPhysix.dir/src/Math.cpp.obj: CMakeFiles/NeoPhysix.dir/includes_CXX.rsp
CMakeFiles/NeoPhysix.dir/src/Math.cpp.obj: C:/Users/Julian/Desktop/neophysix.py/src/Math.cpp
CMakeFiles/NeoPhysix.dir/src/Math.cpp.obj: CMakeFiles/NeoPhysix.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\Julian\Desktop\neophysix.py\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/NeoPhysix.dir/src/Math.cpp.obj"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/NeoPhysix.dir/src/Math.cpp.obj -MF CMakeFiles\NeoPhysix.dir\src\Math.cpp.obj.d -o CMakeFiles\NeoPhysix.dir\src\Math.cpp.obj -c C:\Users\Julian\Desktop\neophysix.py\src\Math.cpp

CMakeFiles/NeoPhysix.dir/src/Math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/NeoPhysix.dir/src/Math.cpp.i"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Julian\Desktop\neophysix.py\src\Math.cpp > CMakeFiles\NeoPhysix.dir\src\Math.cpp.i

CMakeFiles/NeoPhysix.dir/src/Math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/NeoPhysix.dir/src/Math.cpp.s"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Julian\Desktop\neophysix.py\src\Math.cpp -o CMakeFiles\NeoPhysix.dir\src\Math.cpp.s

CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.obj: CMakeFiles/NeoPhysix.dir/flags.make
CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.obj: CMakeFiles/NeoPhysix.dir/includes_CXX.rsp
CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.obj: C:/Users/Julian/Desktop/neophysix.py/src/pyMain.cpp
CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.obj: CMakeFiles/NeoPhysix.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\Julian\Desktop\neophysix.py\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.obj"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.obj -MF CMakeFiles\NeoPhysix.dir\src\pyMain.cpp.obj.d -o CMakeFiles\NeoPhysix.dir\src\pyMain.cpp.obj -c C:\Users\Julian\Desktop\neophysix.py\src\pyMain.cpp

CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.i"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Julian\Desktop\neophysix.py\src\pyMain.cpp > CMakeFiles\NeoPhysix.dir\src\pyMain.cpp.i

CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.s"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Julian\Desktop\neophysix.py\src\pyMain.cpp -o CMakeFiles\NeoPhysix.dir\src\pyMain.cpp.s

CMakeFiles/NeoPhysix.dir/pybind_module.cpp.obj: CMakeFiles/NeoPhysix.dir/flags.make
CMakeFiles/NeoPhysix.dir/pybind_module.cpp.obj: CMakeFiles/NeoPhysix.dir/includes_CXX.rsp
CMakeFiles/NeoPhysix.dir/pybind_module.cpp.obj: C:/Users/Julian/Desktop/neophysix.py/pybind_module.cpp
CMakeFiles/NeoPhysix.dir/pybind_module.cpp.obj: CMakeFiles/NeoPhysix.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\Julian\Desktop\neophysix.py\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/NeoPhysix.dir/pybind_module.cpp.obj"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/NeoPhysix.dir/pybind_module.cpp.obj -MF CMakeFiles\NeoPhysix.dir\pybind_module.cpp.obj.d -o CMakeFiles\NeoPhysix.dir\pybind_module.cpp.obj -c C:\Users\Julian\Desktop\neophysix.py\pybind_module.cpp

CMakeFiles/NeoPhysix.dir/pybind_module.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/NeoPhysix.dir/pybind_module.cpp.i"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Julian\Desktop\neophysix.py\pybind_module.cpp > CMakeFiles\NeoPhysix.dir\pybind_module.cpp.i

CMakeFiles/NeoPhysix.dir/pybind_module.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/NeoPhysix.dir/pybind_module.cpp.s"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Julian\Desktop\neophysix.py\pybind_module.cpp -o CMakeFiles\NeoPhysix.dir\pybind_module.cpp.s

# Object files for target NeoPhysix
NeoPhysix_OBJECTS = \
"CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.obj" \
"CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.obj" \
"CMakeFiles/NeoPhysix.dir/src/Environment.cpp.obj" \
"CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.obj" \
"CMakeFiles/NeoPhysix.dir/src/Math.cpp.obj" \
"CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.obj" \
"CMakeFiles/NeoPhysix.dir/pybind_module.cpp.obj"

# External object files for target NeoPhysix
NeoPhysix_EXTERNAL_OBJECTS =

libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/src/PhysicsEngine.cpp.obj
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/src/PhysicsAPI.cpp.obj
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/src/Environment.cpp.obj
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/src/RigidBody.cpp.obj
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/src/Math.cpp.obj
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/src/pyMain.cpp.obj
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/pybind_module.cpp.obj
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/build.make
libNeoPhysix.dll: C:/Users/Julian/AppData/Local/Programs/Python/Python311/libs/python311.lib
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/linkLibs.rsp
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/objects1.rsp
libNeoPhysix.dll: CMakeFiles/NeoPhysix.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=C:\Users\Julian\Desktop\neophysix.py\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX shared library libNeoPhysix.dll"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\NeoPhysix.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/NeoPhysix.dir/build: libNeoPhysix.dll
.PHONY : CMakeFiles/NeoPhysix.dir/build

CMakeFiles/NeoPhysix.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\NeoPhysix.dir\cmake_clean.cmake
.PHONY : CMakeFiles/NeoPhysix.dir/clean

CMakeFiles/NeoPhysix.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\Julian\Desktop\neophysix.py C:\Users\Julian\Desktop\neophysix.py C:\Users\Julian\Desktop\neophysix.py\build C:\Users\Julian\Desktop\neophysix.py\build C:\Users\Julian\Desktop\neophysix.py\build\CMakeFiles\NeoPhysix.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/NeoPhysix.dir/depend

