//
// Created by zdy on 19-4-1.
//

/**
 *
 */

#include "cinatra.hpp"
#include "json.hpp"

#include <bsoncxx/oid.hpp>
#include <dlib/base64.h>

#include "face_recognition.cpp"
#include "faceset_manage.cpp"


using namespace cinatra;
using json = nlohmann::json;


//日志切面
struct log_t {
    bool before(request &req, response &res) {
        std::cout << "before log -- thread id: " << std::this_thread::get_id() << std::endl;
        return true;
    }

    bool after(request &req, response &res) {
        std::cout << "after log" << std::endl << std::endl;
        return true;
    }
};


/**
 *
 * @param group_id
 */
void build_group_face_index(std::string &group_id) {

    boost::container::stable_vector<std::string> face_infos = faceset::face::getlist(group_id);

    if (face_infos.empty()) {
        return;
    }

    boost::container::stable_vector<std::string> user_ids;
    boost::container::stable_vector<std::vector<double>> feature_list;

    for (std::string face_info_str : face_infos) {

        json face_info_json = json::parse(face_info_str);
        std::string user_id = face_info_json["user_id"];
        std::vector<double> features = face_info_json["descriptor"];

        user_ids.push_back(user_id);
        feature_list.push_back(features);
    }

//    face_infos.clear();

    face::build_group_index(group_id, user_ids, feature_list);

//    std::cout << "user_ids clear ..." << std::endl;
    user_ids.clear();
    boost::container::stable_vector<std::string>().swap(user_ids);
//    std::cout << "user_ids cleared" << std::endl;

//    std::cout << "feature_list clear ..." << std::endl;
    feature_list.clear();
    boost::container::stable_vector<std::vector<double>>().swap(feature_list);
//    std::cout << "feature_list cleared" << std::endl;

//    std::cout << "face_infos clear ..." << std::endl;
    face_infos.clear();
    boost::container::stable_vector<std::string>().swap(face_infos);
//    std::cout << "face_infos cleared" << std::endl;
}


/**
 *
 */
int build_all_face_index() {

    int index_count = 0;

    try {

        boost::container::stable_vector<std::string> group_infos = faceset::group::getlist(0, INT_MAX);
        for (std::string group_info_str : group_infos) {

            json group_info_json = json::parse(group_info_str);
            std::string group_id = group_info_json["group_id"];

            build_group_face_index(group_id);
        }

        index_count = group_infos.size();

        std::cout << "build face index of group count:" << index_count << std::endl;

//        std::cout << "group_infos clear ..." << std::endl;
        group_infos.clear();
        boost::container::stable_vector<std::string>().swap(group_infos);
//        std::cout << "group_infos cleared" << std::endl;

    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
    }

    return index_count;
}


int main() {

    // get CPU logic core num;
    int max_thread_num = std::thread::hardware_concurrency();
    std::cout << "max_thread_num:" << max_thread_num << std::endl;

    // set server work thread num
    http_server server(max_thread_num);

    // set server listen address
    server.listen("0.0.0.0", "8080");


    /**
     * ROOT path
     */
    server.set_http_handler<GET, POST>("/", [](request &req, response &res) {

        json result = json::object();
        result["words"] = "hello face";

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});


    /**
     * rebuild index
     */
    server.set_http_handler<POST>("/face/v1/rebuildIndex", [](request &req, response &res) {

        int index_count = build_all_face_index();

        json result = json::object();
        result["index_count"] = index_count;

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});


    /**
     * rebuild group index
     */
    server.set_http_handler<POST>("/face/v1/rebuildGroupIndex", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

//        std::cout << "param:" << param << std::endl;

            std::string group_id = param["group_id"].get<std::string>();

            build_group_face_index(group_id);

            result["error_code"] = 0;
            result["error_msg"] = "SUCCESS";

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;

            result["error_code"] = 1;
            result["error_msg"] = e.what();
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});


    /**
     * face detect
     */
    server.set_http_handler<POST>("/face/v1/detect", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

//        std::cout << "param:" << param << std::endl;


            std::string image = param["image"].get<std::string>();
            std::string image_type = param["image_type"].get<std::string>();
            std::string face_field = param["face_field"].get<std::string>();
            int max_face_num = param["max_face_num"].get<int>();

            std::string raw_image;

            // decode base64 string
            if ("BASE64" == image_type) {

                long t_start, t_end;
                t_start = utils::timestamp();

                std::ostringstream *sout = new std::ostringstream();
                std::istringstream *sin = new std::istringstream();

                sin->str(image);

                // base64编码器对象
                dlib::base64 base64_coder;
                base64_coder.decode(*sin, *sout);

                t_end = utils::timestamp();
                std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

                std::cout << "base64 size:" << image.size() << std::endl;
                raw_image = sout->str();
                std::cout << "decode size:" << raw_image.size() << std::endl;

                sin->clear();
                delete sin;
                sin = NULL;

                sout->clear();
                delete sout;
                sout = NULL;
            }

            std::string log_id = bsoncxx::oid().to_string();

            dlib::matrix<dlib::rgb_pixel> img;
            face::local_load_image(img, raw_image, log_id);

            raw_image.clear();
            std::string().swap(raw_image);

            // do detect ...
            boost::container::stable_vector<face::FaceInfo *> *faceInfos = new boost::container::stable_vector<face::FaceInfo *>();
            face::face_detect(faceInfos, img, max_face_num);

            dlib::matrix<dlib::rgb_pixel>().swap(img);


//            if (faceInfos.size() > 0) {
//                faceset::image::add(log_id, image);
//            }

            // builder response body

            json face_list_json = json::array();

            for (face::FaceInfo *faceInfo : *faceInfos) {

                std::string face_token = faceInfo->get_face_token();
                double face_probability = faceInfo->get_face_probability();
                std::string label = faceInfo->get_label();
                face::Location *location = faceInfo->get_location();
                boost::container::stable_vector<face::Point *> *landmarks = faceInfo->get_landmarks();

                json landmark_json = json::array();
                for (face::Point *landmark : *landmarks) {

                    json point_json = {{"x", landmark->x},
                                       {"y", landmark->y}};

                    landmark_json.push_back(point_json);
                }

                json face_json = json::object();
                face_json["face_token"] = face_token;
                face_json["face_probability"] = face_probability;
                face_json["label"] = label;
                face_json["location"] = {{"left",     location->left},
                                         {"top",      location->top},
                                         {"width",    location->width},
                                         {"height",   location->height},
                                         {"rotation", location->rotation}};
                face_json["landmark"] = landmark_json;

                face_list_json.push_back(face_json);
            }

            int face_num = faceInfos->size();


            for (face::FaceInfo *faceInfo : *faceInfos) {

                face::Location *location = faceInfo->get_location();
                delete location;
                location = NULL;

                boost::container::stable_vector<face::Point *> *landmarks = faceInfo->get_landmarks();
                for (face::Point *point : *landmarks) {
                    delete point;
                    point = NULL;
                }

                delete landmarks;
                landmarks = NULL;


                boost::container::stable_vector<double> *descriptors = faceInfo->get_descriptors();
                delete descriptors;
                descriptors = NULL;

                delete faceInfo;
                faceInfo = NULL;
            }

            delete faceInfos;
            faceInfos = NULL;


            json result_json = json::object();
            result_json["face_num"] = face_num;
            result_json["face_list"] = face_list_json;

            result["error_code"] = 0;
            result["error_msg"] = "SUCCESS";
            result["log_id"] = log_id;
            result["timestamp"] = utils::timestamp();
            result["cached"] = 0;
            result["result"] = result_json;

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;

            result["error_code"] = 1;
            result["log_id"] = bsoncxx::oid().to_string();
            result["error_msg"] = e.what();
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});



    /**
     * face search
     */
    server.set_http_handler<POST>("/face/v1/search", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

//        std::cout << "param:" << param << std::endl;

            std::string image = param["image"].get<std::string>();
            std::string image_type = param["image_type"].get<std::string>();
            std::string group_id_list = param["group_id_list"].get<std::string>();
            int max_user_num = param["max_user_num"].get<int>();

            std::string raw_image;

            // decode base64 string
            if ("BASE64" == image_type) {

                long t_start, t_end;
                t_start = utils::timestamp();

                std::ostringstream *sout = new std::ostringstream();
                std::istringstream *sin = new std::istringstream();

                sin->str(image);

                // base64编码器对象
                dlib::base64 base64_coder;
                base64_coder.decode(*sin, *sout);

                t_end = utils::timestamp();
                std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

                std::cout << "base64 size:" << image.size() << std::endl;
                raw_image = sout->str();
                std::cout << "decode size:" << raw_image.size() << std::endl;

                sin->clear();
                delete sin;
                sin = NULL;

                sout->clear();
                delete sout;
                sout = NULL;
            }

            std::string log_id = bsoncxx::oid().to_string();

            dlib::matrix<dlib::rgb_pixel> img;
            face::local_load_image(img, raw_image, log_id);

            raw_image.clear();
            std::string().swap(raw_image);

            // do search ...

            boost::container::stable_vector<face::FaceInfo *> *faceInfos = new boost::container::stable_vector<face::FaceInfo *>();
            face::face_detect(faceInfos, img, 1);

            dlib::matrix<dlib::rgb_pixel>().swap(img);


            if ((*faceInfos).size() > 0) {

//                faceset::image::add(log_id, image);

                face::FaceInfo *faceInfo = (*faceInfos)[0];

                std::string face_token = faceInfo->get_face_token();
                boost::container::stable_vector<double> *descriptors = faceInfo->get_descriptors();

                std::vector<string> group_ids = dlib::split(group_id_list, ",");
                for (std::string group_id : group_ids) {
                    std::cout << "group_id:" << group_id << std::endl;
                }


                boost::container::stable_vector<face::UserInfo *> *userInfos = new boost::container::stable_vector<face::UserInfo *>();
                face::user_search(userInfos, descriptors, max_user_num, &group_ids);


                group_ids.clear();
                std::vector<string>().swap(group_ids);


                json user_list_json = json::array();
                for (face::UserInfo *userInfo : *userInfos) {

                    json user_info_json = json::object();

                    boost::container::stable_vector<std::string> user_list = faceset::user::get(userInfo->group_id,
                                                                                                userInfo->user_id);
                    if (user_list.size() > 0) {
                        json user_json = json::parse(user_list[0]);
                        user_info_json = user_json["user_info"];
                    }
                    user_list.clear();
                    boost::container::stable_vector<std::string>().swap(user_list);

                    json user_json = json::object();
                    user_json["group_id"] = userInfo->group_id;
                    user_json["user_id"] = userInfo->user_id;
                    user_json["user_info"] = user_info_json;
                    user_json["score"] = userInfo->score;

                    user_list_json.push_back(user_json);


                    delete userInfo;
                    userInfo = NULL;
                }

                delete userInfos;
                userInfos = NULL;

                result["face_token"] = face_token;
                result["user_list"] = user_list_json;
            }


            for (face::FaceInfo *faceInfo : *faceInfos) {

                face::Location *location = faceInfo->get_location();
                delete location;
                location = NULL;

                boost::container::stable_vector<face::Point *> *landmarks = faceInfo->get_landmarks();
                for (face::Point *point : *landmarks) {
                    delete point;
                    point = NULL;
                }

                delete landmarks;
                landmarks = NULL;


                boost::container::stable_vector<double> *descriptors = faceInfo->get_descriptors();
                delete descriptors;
                descriptors = NULL;

                delete faceInfo;
                faceInfo = NULL;
            }

            delete faceInfos;
            faceInfos = NULL;

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});


    /**
     * face multi-search
     */
    server.set_http_handler<POST>("/face/v1/multi-search", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

//        std::cout << "param:" << param << std::endl;

            std::string image = param["image"].get<std::string>();
            std::string image_type = param["image_type"].get<std::string>();
            std::string group_id_list = param["group_id_list"].get<std::string>();
            int max_face_num = param["max_face_num"].get<int>();
            int max_user_num = param["max_user_num"].get<int>();

            // decode base64 string
            std::string raw_image;

            // decode base64 string
            if ("BASE64" == image_type) {

                long t_start, t_end;
                t_start = utils::timestamp();

                std::ostringstream *sout = new std::ostringstream();
                std::istringstream *sin = new std::istringstream();

                sin->str(image);

                // base64编码器对象
                dlib::base64 base64_coder;
                base64_coder.decode(*sin, *sout);

                t_end = utils::timestamp();
                std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

                std::cout << "base64 size:" << image.size() << std::endl;
                raw_image = sout->str();
                std::cout << "decode size:" << raw_image.size() << std::endl;

                sin->clear();
                delete sin;
                sin = NULL;

                sout->clear();
                delete sout;
                sout = NULL;
            }

            std::string log_id = bsoncxx::oid().to_string();

            dlib::matrix<dlib::rgb_pixel> img;
            face::local_load_image(img, raw_image, log_id);

            raw_image.clear();
            std::string().swap(raw_image);

            // do search ...

            boost::container::stable_vector<face::FaceInfo *> *faceInfos = new boost::container::stable_vector<face::FaceInfo *>();
            face::face_detect(faceInfos, img, max_face_num);

            dlib::matrix<dlib::rgb_pixel>().swap(img);

//            if (faceInfos.size() > 0) {
//                faceset::image::add(log_id, image);
//            }

            json face_list = json::array();

            for (face::FaceInfo *faceInfo : *faceInfos) {

                std::string face_token = faceInfo->get_face_token();
                face::Location *location = faceInfo->get_location();
                boost::container::stable_vector<double> *descriptors = faceInfo->get_descriptors();

                std::vector<string> group_ids = dlib::split(group_id_list, ",");
                for (std::string group_id : group_ids) {
                    std::cout << "group_id:" << group_id << std::endl;
                }


                boost::container::stable_vector<face::UserInfo *> *userInfos = new boost::container::stable_vector<face::UserInfo *>();
                face::user_search(userInfos, descriptors, max_user_num, &group_ids);


                group_ids.clear();
                std::vector<string>().swap(group_ids);


                json user_list_json = json::array();
                for (face::UserInfo *userInfo : *userInfos) {

                    json user_info_json = json::object();

                    boost::container::stable_vector<std::string> user_list = faceset::user::get(userInfo->group_id,
                                                                                                userInfo->user_id);
                    if (user_list.size() > 0) {
                        json user_json = json::parse(user_list[0]);
                        user_info_json = user_json["user_info"];
                    }
                    user_list.clear();
                    boost::container::stable_vector<std::string>().swap(user_list);

                    json user_json = json::object();
                    user_json["group_id"] = userInfo->group_id;
                    user_json["user_id"] = userInfo->user_id;
                    user_json["user_info"] = user_info_json;
                    user_json["score"] = userInfo->score;

                    user_list_json.push_back(user_json);


                    delete userInfo;
                    userInfo = NULL;
                }

                delete userInfos;
                userInfos = NULL;


                json face_json = json::object();
                face_json["face_token"] = face_token;
                face_json["location"] = {{"left",     location->left},
                                         {"top",      location->top},
                                         {"width",    location->width},
                                         {"height",   location->height},
                                         {"rotation", location->rotation}};
                face_json["user_list"] = user_list_json;

                face_list.push_back(face_json);
            }

            int face_num = (*faceInfos).size();


            for (face::FaceInfo *faceInfo : *faceInfos) {

                face::Location *location = faceInfo->get_location();
                delete location;
                location = NULL;

                boost::container::stable_vector<face::Point *> *landmarks = faceInfo->get_landmarks();
                for (face::Point *point : *landmarks) {
                    delete point;
                    point = NULL;
                }

                delete landmarks;
                landmarks = NULL;


                boost::container::stable_vector<double> *descriptors = faceInfo->get_descriptors();
                delete descriptors;
                descriptors = NULL;

                delete faceInfo;
                faceInfo = NULL;
            }

            delete faceInfos;
            faceInfos = NULL;


            json result_json = json::object();
            result_json["face_num"] = face_num;
            result_json["face_list"] = face_list;


            // builder response body

            result["error_code"] = 0;
            result["error_msg"] = "SUCCESS";
            result["log_id"] = bsoncxx::oid().to_string();
            result["timestamp"] = utils::timestamp();
            result["cached"] = 0;
            result["result"] = result_json;


        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;

            result["error_code"] = 1;
            result["log_id"] = bsoncxx::oid().to_string();
            result["error_msg"] = e.what();
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});


    //--------------------------------------------


    /**
     * 创建用户组
     *
     * faceset add group
     */
    server.set_http_handler<POST>("/face/v1/faceset/group/add", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;

            std::string group_id = param["group_id"].get<std::string>();
            std::string group_name = param["group_name"].get<std::string>();


            faceset::group::add(group_id, group_name);

            // builder response body

            result["error_code"] = 0;
            result["error_msg"] = "SUCCESS";
            result["log_id"] = bsoncxx::oid().to_string();

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;

            result["error_code"] = 1;
            result["log_id"] = bsoncxx::oid().to_string();
            result["error_msg"] = e.what();
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 组列表查询
     *
     * faceset group getlist
     */
    server.set_http_handler<POST>("/face/v1/faceset/group/getlist", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;

            int start = param["start"].get<int>();
            int length = param["length"].get<int>();


            boost::container::stable_vector<std::string> groups = faceset::group::getlist(start, length);

            // builder response body

            json group_id_list_json = json::array();
            for (std::string group_json_str : groups) {
                group_id_list_json.push_back(json::parse(group_json_str));
            }

            result["group_id_list"] = group_id_list_json;

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 获取用户列表
     *
     * faceset group getusers
     */
    server.set_http_handler<POST>("/face/v1/faceset/group/getusers", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;

            std::string group_id = param["group_id"].get<std::string>();
            int start = param["start"].get<int>();
            int length = param["length"].get<int>();


            // builder response body

            boost::container::stable_vector<std::string> users = faceset::group::getusers(group_id, start, length);

            // builder response body

            json user_id_list_json = json::array();
            for (std::string user_json_str : users) {
                user_id_list_json.push_back(json::parse(user_json_str));
            }

            users.clear();

            result["user_id_list"] = user_id_list_json;

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 删除用户组
     *
     * faceset group delete
     */
    server.set_http_handler<POST>("/face/v1/faceset/group/delete", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;

            std::string group_id = param["group_id"].get<std::string>();


            faceset::group::delete_one(group_id);


            // builder response body

            result["error_code"] = 0;
            result["log_id"] = bsoncxx::oid().to_string();

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;

            result["error_code"] = 1;
            result["log_id"] = bsoncxx::oid().to_string();
            result["error_msg"] = e.what();
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 人脸注册
     *
     * faceset add user
     */
    server.set_http_handler<POST>("/face/v1/faceset/user/add", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

//            std::cout << "param:" << param << std::endl;

            std::string image = param["image"].get<std::string>();
            std::string image_type = param["image_type"].get<std::string>();
            std::string group_id = param["group_id"].get<std::string>();
            std::string user_id = param["user_id"].get<std::string>();
            std::string user_info = param["user_info"].get<std::string>();
//            std::string quality_control = param["quality_control"].get<std::string>();
//            std::string liveness_control = param["liveness_control"].get<std::string>();
            std::string action_type = param["action_type"].get<std::string>();

            std::string raw_image;

            // decode base64 string
            if ("BASE64" == image_type) {

                long t_start, t_end;
                t_start = utils::timestamp();

                std::ostringstream *sout = new std::ostringstream();
                std::istringstream *sin = new std::istringstream();

                sin->str(image);

                // base64编码器对象
                dlib::base64 base64_coder;
                base64_coder.decode(*sin, *sout);

                t_end = utils::timestamp();
                std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

                std::cout << "base64 size:" << image.size() << std::endl;
                raw_image = sout->str();
                std::cout << "decode size:" << raw_image.size() << std::endl;

                sin->clear();
                delete sin;
                sin = NULL;

                sout->clear();
                delete sout;
                sout = NULL;
            }


            // do add ...

            std::string log_id = bsoncxx::oid().to_string();

            dlib::matrix<dlib::rgb_pixel> img;
            face::local_load_image(img, raw_image, log_id);

            raw_image.clear();
            std::string().swap(raw_image);


            boost::container::stable_vector<face::FaceInfo *> *faceInfos = new boost::container::stable_vector<face::FaceInfo *>();
            face::face_detect(faceInfos, img, 1);

            dlib::matrix<dlib::rgb_pixel>().swap(img);

            if (faceInfos->size() > 0) {

                faceset::image::add(log_id, image, user_id, group_id);

                face::FaceInfo *faceInfo = (*faceInfos)[0];

                std::string face_token = faceInfo->get_face_token();
                face::Location *location = faceInfo->get_location();
                boost::container::stable_vector<face::Point *> *landmarks = faceInfo->get_landmarks();
                boost::container::stable_vector<double> *descriptors = faceInfo->get_descriptors();


                json location_json = {{"left",     location->left},
                                      {"top",      location->top},
                                      {"width",    location->width},
                                      {"height",   location->height},
                                      {"rotation", location->rotation}};

                std::string location_json_str = location_json.dump();


                boost::container::stable_vector<std::string> *landmark_jsons = new boost::container::stable_vector<std::string>();
                for (face::Point *point : *landmarks) {
                    json landmark_json = json::object();
                    landmark_json["x"] = point->x;
                    landmark_json["y"] = point->y;

                    landmark_jsons->push_back(landmark_json.dump());
                }


                if (action_type == "APPEND") {
                    // ...
                } else if (action_type == "REPLACE ") {
                    faceset::user::delete_one(group_id, user_id);
                }

                faceset::user::upsert(group_id, user_id, user_info);
                faceset::face::add(group_id, user_id, log_id, face_token, location_json_str, landmark_jsons,
                                   descriptors);

                delete landmark_jsons;
                landmark_jsons = NULL;

                result["face_token"] = face_token;
                result["location"] = location_json;
            }


            for (face::FaceInfo *faceInfo : *faceInfos) {

                face::Location *location = faceInfo->get_location();
                delete location;
                location = NULL;

                boost::container::stable_vector<face::Point *> *landmarks = faceInfo->get_landmarks();
                for (face::Point *point : *landmarks) {
                    delete point;
                    point = NULL;
                }

                delete landmarks;
                landmarks = NULL;


                boost::container::stable_vector<double> *descriptors = faceInfo->get_descriptors();
                delete descriptors;
                descriptors = NULL;

                delete faceInfo;
                faceInfo = NULL;
            }

            delete faceInfos;
            faceInfos = NULL;

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    }, log_t{});


    /**
     * 人脸更新
     *
     * faceset update user
     */
    server.set_http_handler<POST>("/face/v1/faceset/user/update", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;

            std::string image = param["image"].get<std::string>();
            std::string image_type = param["image_type"].get<std::string>();
            std::string group_id = param["group_id"].get<std::string>();
            std::string user_id = param["user_id"].get<std::string>();
            std::string user_info = param["user_info"].get<std::string>();
//            std::string quality_control = param["quality_control"].get<std::string>();
//            std::string liveness_control = param["liveness_control"].get<std::string>();
            std::string action_type = param["action_type"].get<std::string>();

            std::string raw_image;

            // decode base64 string
            if ("BASE64" == image_type) {

                long t_start, t_end;
                t_start = utils::timestamp();

                std::ostringstream *sout = new std::ostringstream();
                std::istringstream *sin = new std::istringstream();

                sin->str(image);

                // base64编码器对象
                dlib::base64 base64_coder;
                base64_coder.decode(*sin, *sout);

                t_end = utils::timestamp();
                std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

                std::cout << "base64 size:" << image.size() << std::endl;
                raw_image = sout->str();
                std::cout << "decode size:" << raw_image.size() << std::endl;

                sin->clear();
                delete sin;
                sin = NULL;

                sout->clear();
                delete sout;
                sout = NULL;
            }


            // do add ...
            std::string log_id = bsoncxx::oid().to_string();
            dlib::matrix<dlib::rgb_pixel> img;
            face::local_load_image(img, raw_image, log_id);

            raw_image.clear();
            std::string().swap(raw_image);

            boost::container::stable_vector<face::FaceInfo *> *faceInfos = new boost::container::stable_vector<face::FaceInfo *>();
            face::face_detect(faceInfos, img, 1);

            dlib::matrix<dlib::rgb_pixel>().swap(img);

            if ((*faceInfos).size() > 0) {

                faceset::image::add(log_id, image, user_id, group_id);

                face::FaceInfo *faceInfo = (*faceInfos)[0];

                std::string face_token = faceInfo->get_face_token();
                face::Location *location = faceInfo->get_location();
                boost::container::stable_vector<face::Point *> *landmarks = faceInfo->get_landmarks();
                boost::container::stable_vector<double> *descriptors = faceInfo->get_descriptors();


                json location_json = {{"left",     location->left},
                                      {"top",      location->top},
                                      {"width",    location->width},
                                      {"height",   location->height},
                                      {"rotation", location->rotation}};

                std::string location_json_str = location_json.dump();


                boost::container::stable_vector<std::string> *landmark_jsons = new boost::container::stable_vector<std::string>();
                for (face::Point *point : *landmarks) {
                    json landmark_json = json::object();
                    landmark_json["x"] = point->x;
                    landmark_json["y"] = point->y;

                    landmark_jsons->push_back(landmark_json.dump());
                }


                if (action_type == "UPDATE") {
                    boost::container::stable_vector<std::string> rows = faceset::user::get(group_id, user_id);
                    if (rows.size() > 0) {
                        faceset::user::delete_one(group_id, user_id);
                        faceset::user::upsert(group_id, user_id, user_info);
                        faceset::face::add(group_id, user_id, log_id, face_token, location_json_str, landmark_jsons,
                                           descriptors);
                    }
                    rows.clear();
                    boost::container::stable_vector<std::string>().swap(rows);

                } else if (action_type == "REPLACE ") {

                    faceset::user::delete_one(group_id, user_id);
                    faceset::user::upsert(group_id, user_id, user_info);
                    faceset::face::add(group_id, user_id, log_id, face_token, location_json_str, landmark_jsons,
                                       descriptors);
                }


                delete landmark_jsons;
                landmark_jsons = NULL;


                result["face_token"] = face_token;
                result["location"] = location_json;
            }


            for (face::FaceInfo *faceInfo : *faceInfos) {

                face::Location *location = faceInfo->get_location();
                delete location;
                location = NULL;

                boost::container::stable_vector<face::Point *> *landmarks = faceInfo->get_landmarks();
                for (face::Point *point : *landmarks) {
                    delete point;
                    point = NULL;
                }

                delete landmarks;
                landmarks = NULL;


                boost::container::stable_vector<double> *descriptors = faceInfo->get_descriptors();
                delete descriptors;
                descriptors = NULL;

                delete faceInfo;
                faceInfo = NULL;
            }

            delete faceInfos;
            faceInfos = NULL;

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 用户信息查询
     *
     * faceset get user
     */
    server.set_http_handler<POST>("/face/v1/faceset/user/get", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;


            std::string group_id = param["group_id"].get<std::string>();
            std::string user_id = param["user_id"].get<std::string>();


            boost::container::stable_vector<std::string> rows = faceset::user::get(group_id, user_id);

            json user_list_json = json::array();
            for (std::string row : rows) {
                user_list_json.push_back(json::parse(row));
            }

            rows.clear();

            result["user_list"] = user_list_json;

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 复制用户
     *
     * faceset user copy
     */
    server.set_http_handler<POST>("/face/v1/faceset/user/copy", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;


            std::string user_id = param["user_id"].get<std::string>();
            std::string src_group_id = param["src_group_id"].get<std::string>();
            std::string dst_group_id = param["dst_group_id"].get<std::string>();


            faceset::user::copy(user_id, src_group_id, dst_group_id);


            // builder response body

            result["error_code"] = 0;
            result["log_id"] = bsoncxx::oid().to_string();

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;

            result["error_code"] = 1;
            result["log_id"] = bsoncxx::oid().to_string();
            result["error_msg"] = e.what();
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 删除用户
     *
     * faceset user delete
     */
    server.set_http_handler<POST>("/face/v1/faceset/user/delete", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;

            std::string group_id = param["group_id"].get<std::string>();
            std::string user_id = param["user_id"].get<std::string>();


            faceset::user::delete_one(group_id, user_id);


            // builder response body

            result["error_code"] = 0;
            result["log_id"] = bsoncxx::oid().to_string();


        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;

            result["error_code"] = 1;
            result["log_id"] = bsoncxx::oid().to_string();
            result["error_msg"] = e.what();
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 获取用户人脸列表
     *
     * faceset face getlist
     */
    server.set_http_handler<POST>("/face/v1/faceset/face/getlist", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;

            std::string group_id = param["group_id"].get<std::string>();
            std::string user_id = param["user_id"].get<std::string>();


            boost::container::stable_vector<std::string> rows = faceset::face::getlist(group_id, user_id);

            json face_list_json = json::array();
            for (std::string row : rows) {
                face_list_json.push_back(json::parse(row));
            }

            rows.clear();

            result["face_list"] = face_list_json;

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 人脸删除
     *
     * faceset delete face
     */
    server.set_http_handler<POST>("/face/v1/faceset/face/delete", [](request &req, response &res) {

        json result = json::object();

        try {
            // process request body

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;

            std::string group_id = param["group_id"].get<std::string>();
            std::string user_id = param["user_id"].get<std::string>();
            std::string face_token = param["face_token"].get<std::string>();


            faceset::face::delete_one(group_id, user_id, face_token);


            result["error_code"] = 0;
            result["log_id"] = bsoncxx::oid().to_string();

        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;

            result["error_code"] = 1;
            result["log_id"] = bsoncxx::oid().to_string();
            result["error_msg"] = e.what();
        }

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });

    //---------------------------------------------


    /**
     *  close server
     */
    server.set_http_handler<GET, POST>("/close", [&](request &req, response &res) {

        res.set_status_and_content(status_type::ok, "will close");

        server.stop();
    });


    // face recognition ready
    face::init();

    // faceset manage ready
    const std::string uri_str = "mongodb://127.0.0.1:27017";
    faceset::init(uri_str);

    // face search index ready
    build_all_face_index();


    std::cout << "server inited, startup with pid: " << getpid() << std::endl;
    std::cout << std::endl;

    // start server
    server.run();


    return 0;
}