# Define the name of the project
project(landmarktracker)

# This include enable you to use qibuild framework
find_package(qibuild)


# Create an option to make is possible compiling the module


# Create a list of source files
set(_srcs
	myvector.h
    landmarktracker.cpp
    landmarktracker.h
    main.cpp)

  # Create an executable
  qi_create_lib(landmarktracker SHARED ${_srcs} SUBFOLDER naoqi)

# Tell CMake that sayhelloworld depends on ALCOMMON.
# This will set the libraries to link sayhelloworld with,
# the include paths, and so on
qi_use_lib(landmarktracker ALCOMMON BOOST ALMATH)



