FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")#for ubuntu1604
FIND_PACKAGE( OpenCV 2.4 REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS})
# ADD_EXECUTABLE(main main.cpp)

add_executable( generatePointCloud generatePointCloud.cpp )
target_link_libraries( generatePointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_library( slambase slamBase.cpp )
target_link_libraries( slambase
${OpenCV_LIBS}
${PCL_LIBRARIES}
)
add_executable( detectFeatures detectFeatures.cpp )
target_link_libraries(detectFeatures 
    slambase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    )
