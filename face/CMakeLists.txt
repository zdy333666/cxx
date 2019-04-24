cmake_minimum_required(VERSION 3.13)
project(face)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_AVX_INSTRUCTIONS=1") # -std=c++11 -g -Wall -Wno-unused-variable -pthread
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=leak -fno-omit-frame-pointer -Ofast")


# add Boost dependences
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(face ${Boost_LIBRARY_DIRS})

# add cinatra dependences
include_directories("/usr/local/include/cinatra/")

#add annoy dependences
include_directories("/usr/local/include/annoy/")

# Tell cmake we will need dlib.  This command will pull in dlib and compile it
# into your project.  Note that you don't need to compile or install dlib.  All
# cmake needs is the dlib source code folder and it will take care of everything.

#add_subdirectory(/usr/local/include/dlib dlib_build)


# add mongo dependences
include_directories("/usr/local/include/bsoncxx/v_noabi")
include_directories("/usr/local/include/mongocxx/v_noabi")

# add hdf5 dependences
include_directories("/usr/include/hdf5/serial")

# add other dependences
link_directories("/usr/lib/x86_64-linux-gnu/hdf5/serial")

# use std::experimental::filesystem
link_libraries(stdc++fs)


add_executable(face main.cpp)
target_link_libraries(face ${Boost_LIBRARIES} uuid openblas dlib jpeg png gif X11 pthread bsoncxx mongocxx)



add_executable(test_boost test/test_boost.cpp)
target_link_libraries(test_boost ${Boost_LIBRARIES})


add_executable(test_flann_example test/test_flann_example.cpp)
target_link_libraries(test_flann_example hdf5 lz4)

add_executable(test_mongo test/test_mongo.cpp)
target_link_libraries(test_mongo -lbsoncxx -lmongocxx)

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
add_executable(test_build_face_index test/test_build_face_index.cpp)
target_link_libraries(test_build_face_index ${Boost_LIBRARIES} uuid bsoncxx dlib X11 jpeg png gif blas cblas lapack pthread)
