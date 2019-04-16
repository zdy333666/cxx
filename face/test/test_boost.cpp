//
// Created by zdy on 19-4-15.
//

#include <iostream>

#include <boost/array.hpp>
#include <boost/container/stable_vector.hpp>


struct UserInfo {
    std::string group_id;
    std::string user_id;
    double score;
};

struct Location {
    double left;
    double top;
    double width;
    double height;
    long rotation;
};

struct Point {
    double x;
    double y;
};


class FaceInfo {

private:
    std::string face_token;
    double face_probability;
    std::string label;
    Location *location;
    boost::container::stable_vector<Point *> *landmarks;
    boost::container::stable_vector<double> *descriptors;

public:
    FaceInfo() {

    };

    ~FaceInfo() {

        std::cout << "destroy faceInfo" << std::endl;
    }

    void set_face_token(std::string &face_token) {
        this->face_token = face_token;
    }

    std::string get_face_token() {
        return this->face_token;
    }

    void set_face_probability(double face_probability) {
        this->face_probability = face_probability;
    }

    double get_face_probability() {
        return this->face_probability;
    }

    void set_label(std::string &label) {
        this->label = label;
    }

    std::string get_label() {
        return this->label;
    }

    void set_location(Location *location) {
        this->location = location;
    }

    Location *get_location() {
        return this->location;
    }

    void set_landmarks(boost::container::stable_vector<Point *> *landmarks) {
        this->landmarks = landmarks;
    }

    boost::container::stable_vector<Point *> *get_landmarks() {
        return this->landmarks;
    }

    void set_descriptors(boost::container::stable_vector<double> *descriptors) {
        this->descriptors = descriptors;
    }

    boost::container::stable_vector<double> *get_descriptors() {
        return this->descriptors;
    }

};


int main() {

    boost::container::stable_vector<FaceInfo*>* faceInfos = new boost::container::stable_vector<FaceInfo*>();

    for(int i=0;i<5;i++){

        FaceInfo* faceInfo = new FaceInfo();
        faceInfos->push_back(faceInfo);

        std::cout << "push faceInfo " << i << std::endl;
    }

    for(FaceInfo* faceInfo : *faceInfos){
        delete faceInfo;
        faceInfo = NULL;
    }


    delete faceInfos;
    faceInfos = NULL;



//    boost::array<int,3> arr={1,2,3};
//    int* brr = new int[3]{1,2,3};
//
//    brr[0] = 9;
//
//    std::cout << "n:" << brr[0] << std::endl;
//
////    for(int* n:*brr){
////        std::cout << "n:" << n << std::endl;
////    }
//
//    delete[] brr;
//    brr = NULL;
//
//    std::cout << "n:" << brr[0] << std::endl;


    return 0;
}