//
// Created by zdy on 19-4-1.
//

#include <iostream>

//#include <uuid/uuid.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>


int main() {


//    int i,n;
//    uuid_t uu[4];
//    char buf[1024];
//    struct timeval tv;
//    //1、
//    uuid_generate(uu[0]);
//    //2、
//    uuid_generate_random(uu[1]);
//    //3、
//    uuid_generate_time(uu[2]);
//    //4、
//    n = uuid_generate_time_safe(uu[3]);
//    printf("n = %d\n",n);
//    for(i=0;i<4;++i){
//        uuid_unparse(uu[i],buf);
//        printf("uu[%d]\t\t%s\n",i,buf);
//    }
//
//    uuid_time(uu[2],&tv);
//    printf("tv s:%lx  u:%lx\n",tv.tv_sec,tv.tv_usec);



    boost::uuids::uuid a_uuid = boost::uuids::random_generator()(); // 这里是两个() ，因为这里是调用的 () 的运算符重载
    const std::string tmp_uuid = boost::uuids::to_string(a_uuid);


    std::cout << "tmp_uuid:" << tmp_uuid << std::endl;

    return 0;
}