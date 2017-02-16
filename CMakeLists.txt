cmake_minimum_required(VERSION 2.8.3)
project(asr_intermediate_object_generator)

add_definitions(-std=gnu++0x)
add_definitions(-Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  asr_lib_ism
)

find_package(Boost REQUIRED)
find_package(cmake_modules REQUIRED)


find_path(SQLITE3_INCLUDE_DIR sqlite3.h)
find_library(SQLITE3_LIBRARIES sqlite3)




catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SQLITE3_INCLUDE_DIR}
)


add_executable(asr_intermediate_object_generator src/main.cpp
					     src/Domain.cpp 
					     src/Evaluator.cpp
					     src/Object.cpp
					     src/ObjectPoint.cpp
					     src/Scene.cpp)


target_link_libraries(asr_intermediate_object_generator
   ${catkin_LIBRARIES}
   ${SQLITE3_LIBRARIES}
 )
