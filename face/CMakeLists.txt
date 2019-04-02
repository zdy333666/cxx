#OS: Ubuntu 18.04

#install mongoc-driver:
#{
#wget https://github.com/mongodb/mongo-c-driver/releases/download/1.14.0/mongo-c-driver-1.14.0.tar.gz
#tar zxf mongo-c-driver-1.14.0.tar.gz
#cd mongo-c-driver-1.14.0/
#mkdir cmake-build
#cd cmake-build
#cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF -DCMAKE_BUILD_TYPE=Release ..
#sudo make
#sudo make install
#}
#
#
#install mongocxx-driver:
#{
#wget https://github.com/mongodb/mongo-cxx-driver/archive/r3.4.0.tar.gz
#tar zxf mongo-cxx-driver-r3.4.0.tar.gz
#cd mongo-cxx-driver-r3.4.0/
#mkdir cmake-build
#cd cmake-build
#cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
#sudo make
#sudo make install

#ln -s /usr/local/lib/libmongocxx.so /usr/lib/libmongocxx.so

#}


cmake_minimum_required(VERSION 3.13)
project(face)

set(CMAKE_CXX_STANDARD 17)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -uuid")



# add Boost dependences
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(face ${Boost_INCLUDE_DIRS})
link_directories(face ${Boost_LIBRARY_DIRS})


# add cinatra dependences
include_directories("/usr/local/include/cinatra/cinatra")

# add cinatra dependences
include_directories("/usr/local/include/bsoncxx/v_noabi")
#include_directories("/usr/local/include/mongocxx/v_noabi")

# add other dependences
link_libraries(stdc++fs)
link_directories("/usr/local/lib")




add_executable(face main.cpp)
TARGET_LINK_LIBRARIES(face ${Boost_LIBRARIES} pthread uuid bsoncxx)



add_executable(test_uuid test/test_uuid.cpp)
TARGET_LINK_LIBRARIES(test_uuid ${Boost_LIBRARIES} -luuid)

add_executable(test_base64 test/test_base64.cpp)
TARGET_LINK_LIBRARIES(test_base64 ${Boost_LIBRARIES})

add_executable(test_bson_objectId test/test_bson_objectId.cpp)
TARGET_LINK_LIBRARIES(test_bson_objectId ${Boost_LIBRARIES} bsoncxx)

add_executable(test_bson_document test/test_bson_document.cpp)
TARGET_LINK_LIBRARIES(test_bson_document ${Boost_LIBRARIES} bsoncxx)