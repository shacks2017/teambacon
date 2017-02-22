cmake_minimum_required(VERSION 2.8.3)
project(kvc2_path_service)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(kvc2_path_service src/kvc2_path_service2.cpp)
cs_add_executable(kvc2_path_client src/kvc2_path_client.cpp)

#cs_add_executable(path_service_soln src/path_service_soln.cpp)
#cs_add_executable(path_client src/path_client.cpp)

cs_install()
cs_export()
    
