FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )
FIND_PACKAGE( OpenCV REQUIRED )
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS})
# ADD_EXECUTABLE(main main.cpp)
add_executable( generatePointCloud generatePointCloud.cpp )
target_link_libraries( generatePointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )

