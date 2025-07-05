cmake_minimum_required(VERSION 3.10)
project(veloce_lift)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp)
find_package(yaml-cpp)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(Crow REQUIRED)

add_executable(${PROJECT_NAME} "bt.cpp" "app.cpp")
target_include_directories(${PROJECT_NAME} PRIVATE /usr/local/include ${Boost_INCLUDE_DIRS} ${Crow_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  BT::behaviortree_cpp yaml-cpp ${Boost_LIBRARIES} ${Crow_LIBRARIES})