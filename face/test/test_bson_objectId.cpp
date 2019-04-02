//
// Created by zdy on 19-4-2.
//

/*
ObjectId is a 12-byteBSON type,constructed using:

a 4-byte value representing the seconds since the Unix epoch,
a 3-byte machine identifier,
a 2-byte process id, and
a 3-byte counter, starting with a random value.
*/

#include <iostream>

#include <bsoncxx/oid.hpp>


int main() {

//    bsoncxx::oid objectId;
//    std::string objectIdStr =objectId.to_string();


    std::string objectIdStr = bsoncxx::oid().to_string();

    std::cout << "objectId:" << objectIdStr << std::endl;


    return 0;
}