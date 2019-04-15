//
// Created by zdy on 19-4-15.
//

#include <boost/array.hpp>
#include <iostream>

int main(){

    boost::array<int,3> arr={1,2,3};
    int* brr = new int[3]{1,2,3};

    brr[0] = 9;

    std::cout << "n:" << brr[0] << std::endl;

//    for(int* n:*brr){
//        std::cout << "n:" << n << std::endl;
//    }

    delete[] brr;
    brr = NULL;

    std::cout << "n:" << brr[0] << std::endl;


    return 0;
}