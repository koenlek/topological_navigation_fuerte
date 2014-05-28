SET (PROJECT_NAME posest)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)




SET (INCLUDE_DIRS 
"../../eigen3/"
   "../../opencv2/include/"
"../../opencv2/include/opencv"
                  "../../opencv2/modules/core/include"
                  "../../opencv2/modules/imgproc/include"
                  "../../opencv2/modules/video/include"
                  "../../opencv2/modules/features2d/include"
                  "../../opencv2/modules/flann/include"
                  "../../opencv2/modules/objdetect/include"
                  "../../opencv2/modules/calib3d/include"
                  "../../opencv2/modules/ml/include"
                  "../../opencv2/modules/highgui/include"
                  "../../opencv2/modules/legacy/include"
                  "../../opencv2/modules/contrib/include"
"../include"			
            )



 LIST(APPEND CMAKE_LIBRARY_PATH ../../opencv2/build/lib/)

SET(CMAKE_CXX_FLAGS_RELEASE  "-O2 -g -Wall  -msse -msse2 -msse3 -mmmx ")
SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g  -Wall -msse -msse2 -msse3 -mmmx")


# override by appending -DCMAKE_BUILD_TYPE=Debug to cmake argument list
IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
   add_definitions(-DRV_BUILD_RELEASE -DNDEBUG)
ELSE()
  add_definitions()
ENDIF()





SET (LIB_NAMES  opencv_core opencv_highgui opencv_features2d)

FOREACH(lib ${LIB_NAMES})
  FIND_LIBRARY(LIB_${lib} ${lib})

  IF (LIB_${lib})
    MESSAGE(STATUS "found library '${lib}': ${LIB_${lib}}")
  ELSE (LIB_${lib})
    MESSAGE(FATAL_ERROR "cannot find library '${lib}' - this will not work ...")
  ENDIF (LIB_${lib})

  LIST(APPEND LIBS ${LIB_${lib}})
ENDFOREACH(lib)



#Add class XYZ here which consits of two files: XYZ.cpp and XYZ.h




# Set Sources
SET (SOURCES  
../include/posest/cxeigen.hpp 
../include/posest/lie_algebra.hpp
../include/posest/esm.hpp
../include/posest/MatrixFunctions/
../src/esm.cpp
)






INCLUDE_DIRECTORIES( ${INCLUDE_DIRS})
LINK_DIRECTORIES( ${LINK_DIRS})

LINK_LIBRARIES (${PROJECT_NAME} ${LIBS})


ADD_LIBRARY (${PROJECT_NAME} SHARED ${SOURCES} )
ADD_EXECUTABLE(run_esm ../test/run_esm.cpp)

