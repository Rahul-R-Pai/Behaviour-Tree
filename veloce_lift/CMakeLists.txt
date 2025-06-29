cmake_minimum_required(VERSION 3.10.2)

project(veloce_lift)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(behaviortree_cpp)
find_package(yaml-cpp)

add_executable(${PROJECT_NAME} "bt_ui.cpp")
target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp ncurses yaml-cpp)