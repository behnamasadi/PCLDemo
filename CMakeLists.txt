cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

project(PCLDemo)
set(CMAKE_BUILD_TYPE Debug)


if(NOT DEFINED PROCESSOR_COUNT)
  # Unknown:
  set(PROCESSOR_COUNT 0)
 
  # Linux:
  set(cpuinfo_file "/proc/cpuinfo")
  if(EXISTS "${cpuinfo_file}")
    file(STRINGS "${cpuinfo_file}" procs REGEX "^processor.: [0-9]+$")
    list(LENGTH procs PROCESSOR_COUNT)
  endif()
 
  # Mac:
  if(APPLE)
    find_program(cmd_sys_pro "system_profiler")
    if(cmd_sys_pro)
      execute_process(COMMAND ${cmd_sys_pro} OUTPUT_VARIABLE info)
      string(REGEX REPLACE "^.*Total Number Of Cores: ([0-9]+).*$" "\\1"
        PROCESSOR_COUNT "${info}")
    endif()
  endif()
 
  # Windows:
  if(WIN32)
    set(PROCESSOR_COUNT "$ENV{NUMBER_OF_PROCESSORS}")
  endif()
endif()

MESSAGE("Number of Cores: " ${PROCESSOR_COUNT} )

#include ('/home/behnam/usr/share/pcl-1.7/PCLConfig.cmake')

find_package(PCL 1.7 REQUIRED COMPONENTS 
common 
io
kdtree
octree
search 
features 
sample_consensus 
filters 
keypoints 
segmentation
surface 
registration 
visualization 
tracking 
apps 
)

MESSAGE("PCL_FOUND:" ${PCL_FOUND})
MESSAGE("PCL_INCLUDE_DIRS:" ${PCL_LIBRARY_DIRS})
MESSAGE("PCL_LIBRARIES:" ${PCL_LIBRARIES})
MESSAGE("PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
MESSAGE("PCL_VERSION:" ${PCL_VERSION})
MESSAGE("PCL_COMPONENTS:" ${PCL_COMPONENTS})
MESSAGE("PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

ADD_DEFINITIONS(${QT_DEFINITIONS})
SET( CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/src)

#SET(QtDemo_SOURCES src/registration.cpp src/main.cpp )
#SET(QtDemo_HEADERS src/registration.h  )
#SET(QtDemo_FORMS src/registration.ui )


#QT4_WRAP_CPP(QtDemo_HEADERS_MOC ${QtDemo_HEADERS})
#QT4_WRAP_UI(QtDemo_FORMS_HEADERS ${QtDemo_FORMS})
#QT4_ADD_RESOURCES(QtDemo_RESOURCES_RCC ${QtDemo_RESOURCES})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package(HDF5 REQUIRED )
#add_definitions(${HDF5_DEFINITIONS})
#include_directories(${HDF5_INCLUDE_DIRS})
#link_directories(${HDF5_LIBRARIES})

#MESSAGE("HDF5_INCLUDE_DIRS:" ${HDF5_INCLUDE_DIRS})
#MESSAGE("HDF5_LIBRARIES: " ${HDF5_LIBRARIES})
#MESSAGE("HDF5_DEFINITIONS: " ${HDF5_DEFINITIONS})

#INCLUDE(cmake/FindFlann.cmake)
#include_directories(${FLANN_INCLUDE_DIRS})


#target_link_libraries(BasicOperation ${PCL_LIBRARIES} )
#ADD_EXECUTABLE(main ${QtDemo_SOURCES} ${QtDemo_HEADERS_MOC} ${QtDemo_FORMS_HEADERS} )
#TARGET_LINK_LIBRARIES( main  ${QT_LIBRARIES}  ${PCL_LIBRARIES}  )


#ADD_EXECUTABLE(build_tree pcl_junk_src/ClusterRecognitionand6DOFPoseEstimationusingVFHdescriptors/build_tree.cpp )
#TARGET_LINK_LIBRARIES( build_tree ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} )


#ADD_EXECUTABLE(nearest_neighbors pcl_junk_src/ClusterRecognitionand6DOFPoseEstimationusingVFHdescriptors/nearest_neighbors.cpp )
#TARGET_LINK_LIBRARIES( nearest_neighbors ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} )



#ADD_EXECUTABLE(BasicOperation src/BasicOperation.cpp )
#TARGET_LINK_LIBRARIES( BasicOperation ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} )


add_executable(random_sample_consensus src/random_sample_consensus.cpp)
target_link_libraries(random_sample_consensus ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_SEARCH_LIBRARIES}  ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_FILTERS_LIBRARIES}   ${PCL_VISUALIZATION_LIBRARIES}  ${PCL_APPS_LIBRARIES})

add_executable(reading_points src/reading_points.cpp)
target_link_libraries(reading_points ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_SEARCH_LIBRARIES}  ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_FILTERS_LIBRARIES}   ${PCL_VISUALIZATION_LIBRARIES}  ${PCL_APPS_LIBRARIES})


#add_executable(ground_based_rgbd_people_detection src/ground_based_rgbd_people_detection.cpp)
#target_link_libraries(ground_based_rgbd_people_detection ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_SEARCH_LIBRARIES}  ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_FILTERS_LIBRARIES}   ${PCL_VISUALIZATION_LIBRARIES}  ${PCL_APPS_LIBRARIES})

add_executable(pcd_viewer src/pcd_viewer.cpp)
target_link_libraries(pcd_viewer ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}     ${PCL_VISUALIZATION_LIBRARIES}  ${PCL_APPS_LIBRARIES})

add_executable(correspondence_grouping src/correspondence_grouping.cpp)
target_link_libraries(correspondence_grouping ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}     ${PCL_VISUALIZATION_LIBRARIES}  ${PCL_APPS_LIBRARIES})

add_executable(BasicOperation src/BasicOperation.cpp)
target_link_libraries(BasicOperation ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_SEARCH_LIBRARIES}  ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_FILTERS_LIBRARIES}   ${PCL_VISUALIZATION_LIBRARIES}  ${PCL_APPS_LIBRARIES})



#add_executable(pcd_organized_edge_detection src/pcd_organized_edge_detection.cpp)
#target_link_libraries(pcd_organized_edge_detection ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_SEARCH_LIBRARIES}  ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_FILTERS_LIBRARIES}   ${PCL_VISUALIZATION_LIBRARIES}  ${PCL_APPS_LIBRARIES})



add_executable(template_alignment src/template_alignment.cpp)
target_link_libraries(template_alignment ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_SEARCH_LIBRARIES}  ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_FILTERS_LIBRARIES}   ${PCL_VISUALIZATION_LIBRARIES}  ${PCL_APPS_LIBRARIES})



add_executable(tracking_object_in_real_time  src/tracking_object_in_real_time.cpp)
target_link_libraries(tracking_object_in_real_time ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_SEARCH_LIBRARIES}  ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_FILTERS_LIBRARIES}   ${PCL_VISUALIZATION_LIBRARIES}  ${PCL_APPS_LIBRARIES})
