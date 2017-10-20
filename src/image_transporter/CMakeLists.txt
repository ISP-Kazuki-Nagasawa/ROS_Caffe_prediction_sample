cmake_minimum_required(VERSION 2.8)
project(image_transporter)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)

### CXX11
add_compile_options(-std=c++11)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

### Add the publisher
add_executable(publisher src/publisher.cpp)
add_dependencies(publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

### Add the Subscriber
add_executable(subscriber src/subscriber.cpp)
add_dependencies(subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


### Mark executables and/or libraries for installation
install(TARGETS publisher subscriber
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

