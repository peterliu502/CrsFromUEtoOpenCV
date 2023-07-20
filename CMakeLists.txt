# CMakeList.txt : CMake project for CrsFromUEtoOpenCV, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)
set( OpenCV_DIR "D:/ProgramFiles/vcpkg/installed/x64-windows/share/opencv4" )
project ("CrsFromUEtoOpenCV")

# Add opencv to this project.
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add source to this project's executable.
add_executable (CrsFromUEtoOpenCV "CrsFromUEtoOpenCV.cpp" "CrsFromUEtoOpenCV.h")

target_link_libraries( CrsFromUEtoOpenCV ${OpenCV_LIBS} )
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET CrsFromUEtoOpenCV PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
