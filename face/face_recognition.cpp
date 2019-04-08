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
        template<typename> class BN, typename SUBNET>
using residual = add_prev1 <block<N, BN, 1, tag1 < SUBNET>>>;

template<template<int, template<typename> class, int, typename> class block, int N,
        template<typename> class BN, typename SUBNET>
using residual_down = add_prev2 <avg_pool<2, 2, 2, 2, skip1 < tag2 < block<N, BN, 2, tag1 < SUBNET>>>>>>;

template<int N, template<typename> class BN, int stride, typename SUBNET>
using block  = BN<con < N, 3, 3, 1, 1, relu < BN<con < N, 3, 3, stride, stride, SUBNET>>>>>;

template<int N, typename SUBNET> using ares      = relu<residual<block, N, affine, SUBNET>>;
template<int N, typename SUBNET> using ares_down = relu<residual_down<block, N, affine, SUBNET>>;

template<typename SUBNET> using alevel0 = ares_down<256, SUBNET>;
template<typename SUBNET> using alevel1 = ares<256, ares<256, ares_down<256, SUBNET>>>;
template<typename SUBNET> using alevel2 = ares<128, ares<128, ares_down<128, SUBNET>>>;
template<typename SUBNET> using alevel3 = ares<64, ares<64, ares<64, ares_down<64, SUBNET>>>>;
template<typename SUBNET> using alevel4 = ares<32, ares<32, ares<32, SUBNET>>>;

using anet_type = loss_metric <fc_no_bias<128, avg_pool_everything <
                                               alevel0<
                                                       alevel1<
                                                               alevel2<
                                                                       alevel3<
                                                                               alevel4<
                                                                                       max_pool < 3, 3, 2, 2,
                                                                                       relu < affine < con <
                                                                                       32, 7, 7, 2, 2,
                                                                                       input_rgb_image_sized < 150>
                                                                       >>>>>>>>>>>>;

// ----------------------------------------------------------------------------------------

namespace face {

    net_type detector;

    // We will also use a face landmarking model to align faces to a standard pose:  (see face_landmark_detection_ex.cpp for an introduction)
    shape_predictor predictor;

    // And finally we load the DNN responsible for face recognition.
    anet_type descriptor;


    AnnoyIndex<int, double, Euclidean, Kiss32Random> annoy_index = AnnoyIndex<int, double, Euclidean, Kiss32Random>(128);


    std::string people_infos[] = {
            "chendong",
            "liyongliang",
            "shenxiaohui",
            "wangrongfa",
            "wangshanlin",
            "yanpeizong"
    };

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


        std::string annoy_index_path ="../data/precision.tree";
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
    auto face_detect(std::string img_str) try {

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


        t_start = utils::timestamp();

        auto faces = detector(img);

        t_end = utils::timestamp();
        std::cout << "detect face in " << t_end - t_start << " ms" << std::endl;


        json face_list;

        // Run the face detector on the image of our action heroes, and for each face extract a
        // copy that has been normalized to 150x150 pixels in size and appropriately rotated
        // and centered.

        for (auto face : faces) {

            double detection_confidence = face.detection_confidence;
            cout << "face detection_confidence: " << detection_confidence << endl;

            //set face detection confidence threshold
            if(detection_confidence < 1.0){
                continue;
            }

            auto shape = predictor(img, face.rect);

            std::cout << "shape rect:" << shape.get_rect() << std::endl;
            std::cout << "shape size:" << shape.num_parts() << std::endl;


            json landmark_json;
            for (int idx = 0; idx < shape.num_parts(); idx++) {

                auto point = shape.part(idx);

                json point_json = {
                        {"x", point.x()},
                        {"y", point.y()}
                };

                landmark_json.push_back(point_json);
            }

            json face_json;
            face_json["face_token"] = bsoncxx::oid().to_string();
            face_json["face_probability"] = face.detection_confidence;
            face_json["location"] = {
                    {"left",     face.rect.left()},
                    {"top",      face.rect.top()},
                    {"width",    face.rect.width()},
                    {"height",   face.rect.height()},
                    {"rotation", 0},
                    {"label",    face.label}
            };
            face_json["landmark"] = landmark_json;

            face_list.push_back(face_json);
        }


        json result;
        result["face_num"] = faces.size();
        result["face_list"] = face_list;

        return result;

    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
    }


    /**
     *
     * @param img_str
     * @return
     */
    auto face_search(std::string img_str, std::string group_id_list, int max_face_num) try {

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


        t_start = utils::timestamp();

        auto faces = detector(img);

        t_end = utils::timestamp();
        std::cout << "detect face in " << t_end - t_start << " ms" << std::endl;



        // Run the face detector on the image of our action heroes, and for each face extract a
        // copy that has been normalized to 150x150 pixels in size and appropriately rotated
        // and centered.
        std::vector<matrix < rgb_pixel>>
        extracted_faces;
        for (auto face : faces) {

            double detection_confidence = face.detection_confidence;
            cout << "face detection_confidence: " << detection_confidence << endl;

            //set face detection confidence threshold
            if(detection_confidence < 1.0){
                continue;
            }

            auto shape = predictor(img, face.rect);

            matrix <rgb_pixel> face_chip;
            extract_image_chip(img, get_face_chip_details(shape, 150, 0.25), face_chip);

            extracted_faces.push_back(move(face_chip));
        }


        // This call asks the DNN to convert each face image in faces into a 128D vector.
        // In this 128D vector space, images from the same person will be close to each other
        // but vectors from different people will be far apart.  So we can use these vectors to
        // identify if a pair of images are from the same person or from different people.

        t_start = utils::timestamp();

        std::vector<matrix < float, 0, 1>> face_descriptors = descriptor(extracted_faces);

        t_end = utils::timestamp();
        std::cout << "extract descriptors in " << t_end - t_start << " ms" << std::endl;


        json face_list;

        for (int i = 0; i < faces.size(); i++) {
            auto face = faces[i];
            auto face_descriptor = face_descriptors[i];

            std::vector<double> vec(face_descriptor.begin(), face_descriptor.end());


            // search face from annoy index tree
            int result_n = max_face_num;
            int search_k = -1;
            std::vector<int> closest;
            std::vector<double> distances;

            annoy_index.get_nns_by_vector(vec.data(), result_n, search_k, &closest, &distances);

            json user_list_json;
            for(int i = 0; i < closest.size(); i++){
                int id = closest[i];
                double distance = distances[i];

                std::cout << "search result -- id:" << id << "  distance:" << distance << std::endl;

                json user_json;
                user_json["group_id"] = group_id_list;
                user_json["user_id"] = people_infos[id];
                user_json["user_info"] = "userinfo...";
                user_json["score"] = 0.0;

                user_list_json.push_back(user_json);
            }

            closest.clear();
            distances.clear();


            json face_json;
            face_json["face_token"] = bsoncxx::oid().to_string();
            face_json["location"] = {
                    {"left",     face.rect.left()},
                    {"top",      face.rect.top()},
                    {"width",    face.rect.width()},
                    {"height",   face.rect.height()},
                    {"rotation", 0},
                    {"label",    face.label}
            };
            face_json["user_list"] = user_list_json;

            face_list.push_back(face_json);
        }

        json result;
        result["face_num"] = faces.size();
        result["face_list"] = face_list;

        return result;

    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
    }

}
