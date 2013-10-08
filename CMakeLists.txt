cmake_minimum_required (VERSION 2.6)
project (LiveSurface)

#include_directories(/opt/local/include/ImageMagick-6)
#include_directories(/opt/local/include/GraphicsMagick)
#include_directories(/opt/local/include)
include_directories(${CMAKE_SOURCE_DIR}/ms)
include_directories(${CMAKE_SOURCE_DIR}/ms/ann)
include_directories(${CMAKE_SOURCE_DIR}/ms/ngl)

SET(SRCS
  ./main.cpp
  ./texture.cpp
  ./canvas2d.cpp
#  ./canvas3d.cpp
  ./gl2d.cpp
  ./image.cpp
#  ./dijkstra.cpp
  ./dijkstra2.cpp
  ./image_graph.cpp
  ./snode.cpp
)

IF(APPLE)
  ADD_DEFINITIONS(-D__MAC__)
ENDIF(APPLE)

#FIND_PACKAGE(ImageMagick)
FIND_PACKAGE(OpenGL)
FIND_PACKAGE(GLUT)
FIND_PACKAGE(JPEG)
FIND_PACKAGE(PNG)

if (JPEG_FOUND)
include_directories(${JPEG_INCLUDE_DIR})
endif (JPEG_FOUND)
message(STATUS ${JPEG_LIBRARIES})

if (PNG_FOUND)
include_directories(${PNG_INCLUDE_DIR})
message(STATUS ${PNG_LIBRARIES})
endif (PNG_FOUND)

#set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/X11/lib)
#set(CMAKE_LIBRARY_PATH /opt/X11/lib)

#set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/local/lib)
#message(STATUS "${CMAKE_LIBRARY_PATH}")
#FIND_LIBRARY(GM GraphicsMagick )
#FIND_LIBRARY(GMP GraphicsMagick++ )
#message(STATUS "${GM} ${GMP}")

#add_definitions(`/opt/local/bin/GraphicsMagick++-config --cppflags --cxxflags --ldflags --libs`)
#add_definitions(`/opt/local/bin/GraphicsMagick++-config --cppflags`)
add_definitions(-DMAGICKCORE_HDRI_ENABLE=0 -DMAGICKCORE_QUANTUM_DEPTH=16 -DMAGICKCORE_HDRI_ENABLE=0 -DMAGICKCORE_QUANTUM_DEPTH=16 -DMAGICKCORE_HDRI_ENABLE=0 -DMAGICKCORE_QUANTUM_DEPTH=16 -I/opt/local/include/ImageMagick-6)
SET(CMAKE_EXE_LINKER_FLAGS "-L/opt/local/lib -lMagick++-6.Q16 -lMagickWand-6.Q16 -lMagickCore-6.Q16")
SET(CMAKE_SHARED_LINKER_FLAGS "-L/opt/local/lib -lMagick++-6.Q16 -lMagickWand-6.Q16 -lMagickCore-6.Q16")
SET(CMAKE_MODULE_LINKER_FLAGS "-L/opt/local/lib -lMagick++-6.Q16 -lMagickWand-6.Q16 -lMagickCore-6.Q16")

ADD_EXECUTABLE(surface ${SRCS})
TARGET_LINK_LIBRARIES(surface
  ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES}
  ${JPEG_LIBRARIES} ${PNG_LIBRARIES} ${LIBS}
  /opt/X11/lib/libX11.dylib)

