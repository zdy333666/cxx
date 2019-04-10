//
// Created by zdy on 19-4-6.
//

#include <dlib/dnn.h>
#include <dlib/data_io.h>
#include <dlib/image_io.h>

#include <bsoncxx/oid.hpp>
#include "nlohmann_json.hpp"
#include "utils.cpp"

#include "kissrandom.h"
#include "annoylib.h"

using namespace std;
using namespace dlib;
using json = nlohmann::json;

// ----------------------------------------------------------------------------------------

template<long num_filters, typename SUBNET> using con5d = con<num_filters, 5, 5, 2, 2, SUBNET>;
template<long num_filters, typename SUBNET> using con5  = con<num_filters, 5, 5, 1, 1, SUBNET>;

template<typename SUBNET> using downsampler  = relu <affine<con5d<32,
        relu < affine < con5d<32, relu < affine < con5d<16, SUBNET>>>>>>>>>;
template<typename SUBNET> using rcon5  = relu <affine<con5<45, SUBNET>>>;

using net_type = loss_mmod <con<1, 9, 9, 1, 1, rcon5<rcon5<rcon5<downsampler<
        input_rgb_image_pyramid < pyramid_down < 6>>>>>>>>;

// ----------------------------------------------------------------------------------------

// The next bit of code defines a ResNet network.  It's basically copied
// and pasted from the dnn_imagenet_ex.cpp example, except we replaced the loss
// layer with loss_metric and made the network somewhat smaller.  Go read the introductory
// dlib DNN examples to learn what all this stuff means.
//
// Also, the dnn_metric_learning_on_images_ex.cpp example shows how to train this network.
// The dlib_face_recognition_resnet_model_v1 model used by this example was trained using
// essentially the code shown in dnn_metric_learning_on_images_ex.cpp except the
// mini-batches were made larger (35x15 instead of 5x5), the iterations without progress
// was set to 10000, and the training dataset consisted of about 3 million images instead of
// 55.  Also, the input layer was locked to images of size 150.
template<template<int, template<typename> class, int, typename> class block, int N,
        template<typename> class BN, typename SUBNET> using residual = add_prev1 <block<N, BN, 1, tag1 < SUBNET>>>;

template<template<int, template<typename> class, int, typename> class block, int N,
        template<typename> class BN, typename SUBNET> using residual_down = add_prev2 <avg_pool<2, 2, 2, 2,
        skip1 < tag2 < block<N, BN, 2, tag1 < SUBNET>>>>>>;

template<int N, template<typename> class BN, int stride, typename SUBNET> using block  = BN<
        con < N, 3, 3, 1, 1, relu < BN<con < N, 3, 3, stride, stride, SUBNET>>>>>;

template<int N, typename SUBNET> using ares      = relu<residual<block, N, affine, SUBNET>>;
template<int N, typename SUBNET> using ares_down = relu<residual_down<block, N, affine, SUBNET>>;

template<typename SUBNET> using alevel0 = ares_down<256, SUBNET>;
template<typename SUBNET> using alevel1 = ares<256, ares<256, ares_down<256, SUBNET>>>;
template<typename SUBNET> using alevel2 = ares<128, ares<128, ares_down<128, SUBNET>>>;
template<typename SUBNET> using alevel3 = ares<64, ares<64, ares<64, ares_down<64, SUBNET>>>>;
template<typename SUBNET> using alevel4 = ares<32, ares<32, ares<32, SUBNET>>>;

using anet_type = loss_metric <fc_no_bias<128, avg_pool_everything < alevel0<alevel1<alevel2<alevel3<alevel4<
        max_pool < 3, 3, 2, 2, relu < affine < con < 32, 7, 7, 2, 2, input_rgb_image_sized < 150>
>>>>>>>>>>>>;

// ----------------------------------------------------------------------------------------


namespace face {

    struct UserInfo {
        std::string group_id;
        std::string user_id;
        std::string user_info;
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
        Location location;
        std::vector<Point> landmarks;
        std::vector<double> descriptors;

    public:
        FaceInfo() {

        };

        ~FaceInfo() {

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

        void set_location(Location &location) {
            this->location = location;
        }

        Location get_location() {
            return this->location;
        }

        void set_landmarks(std::vector<Point> &landmarks) {
            this->landmarks = landmarks;
        }

        std::vector<Point> get_landmarks() {
            return this->landmarks;
        }

        void set_descriptors(std::vector<double> &descriptors) {
            this->descriptors = descriptors;
        }

        std::vector<double> get_descriptors() {
            return this->descriptors;
        }

    };


    // ----------------------------------------------------------------------------------------

    net_type detector;

    // We will also use a face landmarking model to align faces to a standard pose:  (see face_landmark_detection_ex.cpp for an introduction)
    shape_predictor predictor;

    // And finally we load the DNN responsible for face recognition.
    anet_type descriptor;


    AnnoyIndex<int, double, Euclidean, Kiss32Random> annoy_index = AnnoyIndex<int, double, Euclidean, Kiss32Random>(
            128);


    std::string people_infos[] = {"chendong", "liyongliang", "shenxiaohui", "wangrongfa", "wangshanlin", "yanpeizong"};

    /**
     *
     */
    void init() {
        std::string detector_model_path = "../data/mmod_human_face_detector.dat";
        dlib::deserialize(detector_model_path) >> detector;
        std::cout << "loaded detector model file:" << detector_model_path << std::endl;

        std::string predictor_model_path = "../data/shape_predictor_68_face_landmarks.dat";
        deserialize(predictor_model_path) >> predictor;
        std::cout << "loaded predictor model file:" << predictor_model_path << std::endl;

        std::string descriptor_model_path = "../data/dlib_face_recognition_resnet_model_v1.dat";
        deserialize(descriptor_model_path) >> descriptor;
        std::cout << "loaded descriptor model file:" << descriptor_model_path << std::endl;


        std::string annoy_index_path = "../data/faceset_test.tree";
        annoy_index.load(annoy_index_path.c_str());
        std::cout << "loaded annoy index file:" << annoy_index_path << std::endl;

        std::cout << "annoy_index_n_items: " << annoy_index.get_n_items() << std::endl;
        std::cout << "annoy_index_n_trees: " << annoy_index.get_n_trees() << std::endl;
    }


    /**
     * 
     * @param img_str 
     * @return 
     */
    dlib::matrix<dlib::rgb_pixel> local_load_image(std::string &img_str) {

        long t_start, t_end;
        t_start = utils::timestamp();

        std::string file_id = bsoncxx::oid().to_string();

        std::string temp_file_path = "../temp/" + file_id + ".jpg";

        std::ofstream tmp_file;
        tmp_file.open(temp_file_path);

        if (!tmp_file.is_open()) {
            std::cout << "can not access temp file:" << temp_file_path << std::endl;
            exit(1);
        }

        tmp_file.write(img_str.data(), img_str.size());
        tmp_file.close();

        dlib::matrix<dlib::rgb_pixel> img;
        dlib::load_image(img, temp_file_path);

        t_end = utils::timestamp();
        std::cout << "save temp image in " << t_end - t_start << " ms" << std::endl;

        return img;
    }

    /**
     * 
     * @param img 
     * @return 
     */
    auto local_get_faces(dlib::matrix<dlib::rgb_pixel> &img) {

        long t_start, t_end;
        t_start = utils::timestamp();

        auto faces = detector(img);

        t_end = utils::timestamp();
        std::cout << "detect face in " << t_end - t_start << " ms" << std::endl;

        return faces;
    }

    /**
     * 
     */
    std::vector<matrix < float, 0, 1>> local_get_descriptors(std::vector<matrix < rgb_pixel>> &faces) {

        long t_start, t_end;
        t_start = utils::timestamp();

        // This call asks the DNN to convert each face image in faces into a 128D vector.
        // In this 128D vector space, images from the same person will be close to each other
        // but vectors from different people will be far apart.  So we can use these vectors to
        // identify if a pair of images are from the same person or from different people.

        std::vector<matrix < float, 0, 1>> descriptors = descriptor(faces);

        t_end = utils::timestamp();
        std::cout << "extract descriptors in " << t_end - t_start << " ms" << std::endl;

        return descriptors;
    }


    /**
     * 
     * @param descriptor 
     * @param result_n 
     * @return 
     */
std::vector<int> local_ann_search(double *descriptor, int result_n) {

    // search face from annoy index tree
    int search_k = -1;
    std::vector<int> closest;
    std::vector<double> distances;

    annoy_index.get_nns_by_vector(descriptor, result_n, search_k, &closest, &distances);

    for (int i = 0; i < closest.size(); i++) {
        int id = closest[i];
        double distance = distances[i];

        std::cout << "search result -- id:" << id << "  distance:" << distance << std::endl;
    }

    distances.clear();


    return closest;
}


/**
 * 
 * @param descriptor 
 * @param result_n 
 * @param group_id 
 * @return 
 */
std::vector<UserInfo> user_search(double *descriptor, int result_n, std::string &group_id) {

    std::vector<int> closest = local_ann_search(descriptor, result_n);

    std::vector<UserInfo> userInfos;
    for (int i = 0; i < closest.size(); i++) {
        int id = closest[i];

        UserInfo userInfo;
        userInfo.group_id = group_id;
        userInfo.user_id = people_infos[id];
        userInfo.score = 0.0;

        userInfos.push_back(userInfo);
    }

    return userInfos;
}


/**
*
* @param img_str
* @return
*/
std::vector<FaceInfo> face_detect(std::string img_str, int max_face_num) try {

    dlib::matrix<dlib::rgb_pixel> img = local_load_image(img_str);

    auto faces = local_get_faces(img);


    // Run the face detector on the image of our action heroes, and for each face extract a
    // copy that has been normalized to 150x150 pixels in size and appropriately rotated
    // and centered.
    std::vector<matrix<rgb_pixel>> extracted_faces;
    for (auto face : faces) {

        double detection_confidence = face.detection_confidence;
        cout << "face detection_confidence: " << detection_confidence << endl;

        //set face detection confidence threshold
        if (detection_confidence < 1.0) {
            continue;
        }

        auto shape = predictor(img, face.rect);

        matrix<rgb_pixel> face_chip;
        extract_image_chip(img, get_face_chip_details(shape, 150, 0.25), face_chip);

        extracted_faces.push_back(move(face_chip));
    }


    std::vector<matrix<float, 0, 1>> face_descriptors = local_get_descriptors(extracted_faces);


    std::vector<FaceInfo> faceInfos;

    for (int i = 0; i < faces.size(); i++) {

        auto face = faces[i];
        auto face_descriptor = face_descriptors[i];

        auto shape = predictor(img, face.rect);
        std::cout << "shape rect:" << shape.get_rect() << std::endl;
        std::cout << "shape size:" << shape.num_parts() << std::endl;

        std::string face_token = bsoncxx::oid().to_string();
        double face_probability = face.detection_confidence;
        Location location;
        std::vector<Point> landmarks;
        std::vector<double> descriptors(face_descriptor.begin(), face_descriptor.end());

        location.left = face.rect.left();
        location.top = face.rect.top();
        location.width = face.rect.width();
        location.height = face.rect.height();
        location.rotation = 0;

        for (int idx = 0; idx < shape.num_parts(); idx++) {
            auto part = shape.part(idx);

            Point point;
            point.x = part.x();
            point.y = part.y();

            landmarks.push_back(point);
        }

        face::FaceInfo faceInfo;
        faceInfo.set_face_token(face_token);
        faceInfo.set_face_probability(face_probability);
        faceInfo.set_location(location);
        faceInfo.set_landmarks(landmarks);
        faceInfo.set_descriptors(descriptors);

        faceInfos.push_back(faceInfo);
    }

    return faceInfos;

} catch (std::exception &e) {
    std::cout << e.what() << std::endl;
}


}
