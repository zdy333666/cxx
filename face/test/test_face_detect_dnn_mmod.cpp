//
// Created by zdy on 19-4-6.
//

#include <dlib/dnn.h>
#include <dlib/data_io.h>
#include <dlib/image_io.h>
#include <bsoncxx/oid.hpp>

#include "nlohmann_json.hpp"

#include "../utils.cpp"

using namespace std;
using namespace dlib;
using json = nlohmann::json;

// ----------------------------------------------------------------------------------------

template<long num_filters, typename SUBNET> using con5d = con<num_filters, 5, 5, 2, 2, SUBNET>;
template<long num_filters, typename SUBNET> using con5  = con<num_filters, 5, 5, 1, 1, SUBNET>;

template<typename SUBNET> using downsampler  = relu<affine<con5d<32,
        relu<affine<con5d<32, relu<affine<con5d<16, SUBNET>>>>>>>>>;
template<typename SUBNET> using rcon5  = relu<affine<con5<45, SUBNET>>>;

using net_type = loss_mmod<con<1, 9, 9, 1, 1, rcon5<rcon5<rcon5<downsampler<
        input_rgb_image_pyramid<pyramid_down<6>>>>>>>>;

// ----------------------------------------------------------------------------------------

net_type net;

namespace face {

    void init() {
        std::string modelPath = "../data/mmod_human_face_detector.dat";
        dlib::deserialize(modelPath) >> net;
        std::cout << "loaded model file:" << modelPath << std::endl;
    }


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

        auto faces = net(img);

        t_end = utils::timestamp();
        std::cout << "detect face in " << t_end - t_start << " ms" << std::endl;


        json face_list;
        for (auto face : faces) {

            std::cout << "face:" << face << std::endl;

            json face_json;
            face_json["face_token"] = bsoncxx::oid().to_string();
            face_json["face_probability"] = face.detection_confidence;
            face_json["location"] ={
                    {"left", face.rect.left()},
                    {"top", face.rect.top()},
                    {"width", face.rect.width()},
                    {"height", face.rect.height()},
                    {"rotation", 0},
                    {"label", face.label}
            };

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
