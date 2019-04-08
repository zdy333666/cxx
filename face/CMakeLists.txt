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


include_directories("/home/zdy/git-local/annoy/src")

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
target_link_libraries(face ${Boost_LIBRARIES} -luuid -lbsoncxx -ldlib -lX11 -ljpeg -lpng -lgif -lblas -lcblas -llapack -lpthread)



#add_executable(test_uuid test/test_uuid.cpp)
#target_link_libraries(test_uuid ${Boost_LIBRARIES} -luuid)
#
#add_executable(test_base64 test/test_base64.cpp)
#target_link_libraries(test_base64 ${Boost_LIBRARIES})
#
#add_executable(test_bson_objectId test/test_bson_objectId.cpp)
#target_link_libraries(test_bson_objectId ${Boost_LIBRARIES} bsoncxx)
#
#add_executable(test_bson_document test/test_bson_document.cpp)
#target_link_libraries(test_bson_document ${Boost_LIBRARIES} bsoncxx)
#
#add_executable(test_annoy test/test_annoy.cpp)
#
#add_executable(test_annoy_precision test/test_annoy_precision.cpp)
#
#add_executable(test_build_face_index test/test_build_face_index.cpp)
#target_link_libraries(test_build_face_index ${Boost_LIBRARIES} -luuid -lbsoncxx -ldlib -lX11 -ljpeg -lpng -lgif -lblas -lcblas -llapack -lpthread)
