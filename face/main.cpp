//
// Created by zdy on 19-4-1.
//

/**
 *  https://github.com/qicosmos/cinatra
 */

#include "http_server.hpp"
#include "nlohmann_json.hpp"

#include <bsoncxx/oid.hpp>
#include <dlib/base64.h>

#include "face_recognition.cpp"
#include "faceset_manage.cpp"


using namespace cinatra;
using json = nlohmann::json;


//日志切面
struct log_t {
    bool before(request &req, response &res) {
        std::cout << "before log" << std::endl;
        return true;
    }

    bool after(request &req, response &res) {
        std::cout << "after log" << std::endl << std::endl;
        return true;
    }
};


int main() {

    // get CPU logic core num;
    int max_thread_num = std::thread::hardware_concurrency();
    std::cout << "max_thread_num:" << max_thread_num << std::endl;

    // set server work thread num
    http_server server(max_thread_num);

    // set server listen address
    server.listen("0.0.0.0", "8080");


    server.set_http_handler<GET, POST>("/", [](request &req, response &res) {
        res.set_status_and_content(status_type::ok, "hello world");
    }, log_t{});


    server.set_http_handler<GET, POST>("/test/json", [](request &req, response &res) {

        json result;
        result["name"] = "tom";
        result["age"] = 20;
        result["list"] = {10, 20, 30};

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});



    /**
     * face detect
     */
    server.set_http_handler<GET, POST>("/face/v1/detect", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

//        std::cout << "param:" << param << std::endl;


        std::string image = param["image"].get<std::string>();
        std::string image_type = param["image_type"].get<std::string>();
        std::string face_field = param["face_field"].get<std::string>();
        int max_face_num = param["max_face_num"].get<int>();


        // decode base64 string
        if ("BASE64" == image_type) {

            long t_start, t_end;
            t_start = utils::timestamp();

            std::ostringstream sout;
            std::istringstream sin;

            sin.str(image);

            // base64编码器对象
            dlib::base64 base64_coder;
            base64_coder.decode(sin, sout);

            t_end = utils::timestamp();
            std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

            std::cout << "base64 size:" << image.size() << std::endl;
            image = sout.str();
            std::cout << "decode size:" << image.size() << std::endl;

            sin.clear();
            sout.clear();
        }


        // do detect ...
        std::vector<face::FaceInfo> faceInfos = face::face_detect(image, max_face_num);


        // builder response body

        json face_list_json;

        for (face::FaceInfo faceInfo : faceInfos) {
            std::string face_token = faceInfo.get_face_token();
            double face_probability = faceInfo.get_face_probability();
            std::string label = faceInfo.get_label();
            face::Location location = faceInfo.get_location();
            std::vector<face::Point> landmarks = faceInfo.get_landmarks();

            json landmark_json;
            for (face::Point landmark : landmarks) {

                json point_json = {{"x", landmark.x},
                                   {"y", landmark.y}};

                landmark_json.push_back(point_json);
            }

            json face_json;
            face_json["face_token"] = face_token;
            face_json["face_probability"] = face_probability;
            face_json["label"] = label;
            face_json["location"] = {{"left",     location.left},
                                     {"top",      location.top},
                                     {"width",    location.width},
                                     {"height",   location.height},
                                     {"rotation", location.rotation}};
            face_json["landmark"] = landmark_json;

            face_list_json.push_back(face_json);
        }

        json result_json;
        result_json["face_num"] = faceInfos.size();
        result_json["face_list"] = face_list_json;

        json result;
        result["error_code"] = 0;
        result["error_msg"] = "SUCCESS";
        result["log_id"] = bsoncxx::oid().to_string();
        result["timestamp"] = utils::timestamp();
        result["cached"] = 0;
        result["result"] = result_json;

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});



    /**
     * face search
     */
    server.set_http_handler<GET, POST>("/face/v1/search", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

//        std::cout << "param:" << param << std::endl;

        std::string image = param["image"].get<std::string>();
        std::string image_type = param["image_type"].get<std::string>();
        std::string group_id_list = param["group_id_list"].get<std::string>();
        int max_user_num = param["max_user_num"].get<int>();

        // decode base64 string
        if ("BASE64" == image_type) {

            long t_start, t_end;
            t_start = utils::timestamp();

            std::ostringstream sout;
            std::istringstream sin;

            sin.str(image);

            // base64编码器对象
            dlib::base64 base64_coder;
            base64_coder.decode(sin, sout);

            t_end = utils::timestamp();
            std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

            std::cout << "base64 size:" << image.size() << std::endl;
            image = sout.str();
            std::cout << "decode size:" << image.size() << std::endl;

            sin.clear();
            sout.clear();
        }


        // do search ...

        std::vector<face::FaceInfo> faceInfos = face::face_detect(image, 1);

        json result;

        if (faceInfos.size() > 0) {

            face::FaceInfo faceInfo = faceInfos[0];

            std::string face_token = faceInfo.get_face_token();
            std::vector<double> descriptors = faceInfo.get_descriptors();

            std::vector<face::UserInfo> userInfos = face::user_search(descriptors.data(), max_user_num, group_id_list);

            json user_list_json;
            for (face::UserInfo userInfo : userInfos) {
                json user_json;
                user_json["group_id"] = userInfo.group_id;
                user_json["user_id"] = userInfo.user_id;
                user_json["user_info"] = userInfo.user_info;
                user_json["score"] = userInfo.score;

                user_list_json.push_back(user_json);
            }

            result["face_token"] = face_token;
            result["user_list"] = user_list_json;
        }


        // builder response body

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});


    /**
     * face multi-search
     */
    server.set_http_handler<GET, POST>("/face/v1/multi-search", [](request &req, response &res) {

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
        if ("BASE64" == image_type) {

            long t_start, t_end;
            t_start = utils::timestamp();

            std::ostringstream sout;
            std::istringstream sin;

            sin.str(image);

            // base64编码器对象
            dlib::base64 base64_coder;
            base64_coder.decode(sin, sout);

            t_end = utils::timestamp();
            std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

            std::cout << "base64 size:" << image.size() << std::endl;
            image = sout.str();
            std::cout << "decode size:" << image.size() << std::endl;

            sin.clear();
            sout.clear();
        }


        // do search ...

        std::vector<face::FaceInfo> faceInfos = face::face_detect(image, max_face_num);

        json face_list;

        for (face::FaceInfo faceInfo : faceInfos) {
            std::string face_token = faceInfo.get_face_token();
            face::Location location = faceInfo.get_location();
            std::vector<double> descriptors = faceInfo.get_descriptors();

            std::vector<face::UserInfo> userInfos = face::user_search(descriptors.data(), max_user_num, group_id_list);

            json user_list_json;
            for (face::UserInfo userInfo : userInfos) {
                json user_json;
                user_json["group_id"] = userInfo.group_id;
                user_json["user_id"] = userInfo.user_id;
                user_json["user_info"] = userInfo.user_info;
                user_json["score"] = userInfo.score;

                user_list_json.push_back(user_json);
            }

            json face_json;
            face_json["face_token"] = face_token;
            face_json["location"] = {{"left",     location.left},
                                     {"top",      location.top},
                                     {"width",    location.width},
                                     {"height",   location.height},
                                     {"rotation", location.rotation}};
            face_json["user_list"] = user_list_json;

            face_list.push_back(face_json);
        }

        json result_json;
        result_json["face_num"] = faceInfos.size();
        result_json["face_list"] = face_list;


        // builder response body

        json result;
        result["error_code"] = 0;
        result["error_msg"] = "SUCCESS";
        result["log_id"] = bsoncxx::oid().to_string();
        result["timestamp"] = utils::timestamp();
        result["cached"] = 0;
        result["result"] = result_json;

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});


    //--------------------------------------------


    /**
     * 创建用户组
     *
     * faceset add group
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/group/add", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;

        std::string group_id = param["group_id"].get<std::string>();
        std::string group_name = param["group_name"].get<std::string>();


        faceset::group::add(group_id, group_name);

        // builder response body

        json result;
        result["error_code"] = 0;
        result["error_msg"] = "SUCCESS";
        result["log_id"] = bsoncxx::oid().to_string();

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 组列表查询
     *
     * faceset group getlist
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/group/getlist", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;

        int start = param["start"].get<int>();
        int length = param["length"].get<int>();


        std::vector<std::string> groups = faceset::group::getlist(start, length);


        // builder response body

        json group_id_list_json;
        for (std::string group_json_str : groups) {
            group_id_list_json.push_back(group_json_str);
        }

        json result;
        result["group_id_list"] = group_id_list_json;

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 获取用户列表
     *
     * faceset group getusers
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/group/getusers", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;

        std::string group_id = param["group_id"].get<std::string>();
        int start = param["start"].get<int>();
        int length = param["length"].get<int>();


        // builder response body

        std::vector<std::string> users = faceset::group::getusers(group_id, start, length);

        // builder response body

        json user_id_list_json;
        for (std::string user_json_str : users) {
            user_id_list_json.push_back(user_json_str);
        }

        json result;
        result["user_id_list"] = user_id_list_json;

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 删除用户组
     *
     * faceset group delete
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/group/delete", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;

        std::string group_id = param["group_id"].get<std::string>();


        faceset::group::delete_one(group_id);


        // builder response body

        json result;
        result["error_code"] = 0;
        result["log_id"] = bsoncxx::oid().to_string();

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 人脸注册
     *
     * faceset add user
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/user/add", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;

        std::string image = param["image"].get<std::string>();
        std::string image_type = param["image_type"].get<std::string>();
        std::string group_id = param["group_id"].get<std::string>();
        std::string user_id = param["user_id"].get<std::string>();
        std::string user_info = param["user_info"].get<std::string>();
        std::string quality_control = param["quality_control"].get<std::string>();
        std::string liveness_control = param["liveness_control"].get<std::string>();
        std::string action_type = param["action_type"].get<std::string>();

        // decode base64 string
        if ("BASE64" == image_type) {

            long t_start, t_end;
            t_start = utils::timestamp();

            std::ostringstream sout;
            std::istringstream sin;

            sin.str(image);

            // base64编码器对象
            dlib::base64 base64_coder;
            base64_coder.decode(sin, sout);

            t_end = utils::timestamp();
            std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

            std::cout << "base64 size:" << image.size() << std::endl;
            image = sout.str();
            std::cout << "decode size:" << image.size() << std::endl;

            sin.clear();
            sout.clear();
        }



        // do add ...


        // builder response body

        json result;


        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 人脸更新
     *
     * faceset update user
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/user/update", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;

        std::string image = param["image"].get<std::string>();
        std::string image_type = param["image_type"].get<std::string>();
        std::string group_id = param["group_id"].get<std::string>();
        std::string user_id = param["user_id"].get<std::string>();
        std::string user_info = param["user_info"].get<std::string>();
        std::string quality_control = param["quality_control"].get<std::string>();
        std::string liveness_control = param["liveness_control"].get<std::string>();
        std::string action_type = param["action_type"].get<std::string>();

        // decode base64 string
        if ("BASE64" == image_type) {

            long t_start, t_end;
            t_start = utils::timestamp();

            std::ostringstream sout;
            std::istringstream sin;

            sin.str(image);

            // base64编码器对象
            dlib::base64 base64_coder;
            base64_coder.decode(sin, sout);

            t_end = utils::timestamp();
            std::cout << "decode base64 image in " << t_end - t_start << " ms" << std::endl;

            std::cout << "base64 size:" << image.size() << std::endl;
            image = sout.str();
            std::cout << "decode size:" << image.size() << std::endl;

            sin.clear();
            sout.clear();
        }



        // do add ...


        // builder response body

        json result;


        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 用户信息查询
     *
     * faceset get user
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/user/get", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;


        std::string group_id = param["group_id"].get<std::string>();
        std::string user_id = param["user_id"].get<std::string>();



        // builder response body

        json result;


        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 复制用户
     *
     * faceset user copy
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/user/copy", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;


        std::string user_id = param["user_id"].get<std::string>();
        std::string src_group_id = param["src_group_id"].get<std::string>();
        std::string dst_group_id = param["dst_group_id"].get<std::string>();



        // builder response body

        json result;


        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 删除用户
     *
     * faceset user delete
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/user/delete", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;


        std::string group_id = param["group_id"].get<std::string>();
        std::string user_id = param["user_id"].get<std::string>();



        // builder response body

        json result;


        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 获取用户人脸列表
     *
     * faceset face getlist
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/face/getlist", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;

        std::string group_id = param["group_id"].get<std::string>();
        std::string user_id = param["user_id"].get<std::string>();


        json result;
        result["error_code"] = 0;
        result["error_msg"] = "SUCCESS";
        result["log_id"] = std::rand();


        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());
    });


    /**
     * 人脸删除
     *
     * faceset delete face
     */
    server.set_http_handler<GET, POST>("/face/v1/faceset/face/delete", [](request &req, response &res) {

        // process request body

        std::string_view body = req.body();
        json param = json::parse(body);

        std::cout << "param:" << param << std::endl;

        std::string group_id = param["group_id"].get<std::string>();
        std::string user_id = param["user_id"].get<std::string>();
        std::string face_token = param["face_token"].get<std::string>();


        json result;
        result["error_code"] = 0;
        result["error_msg"] = "SUCCESS";
        result["log_id"] = std::rand();


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


    face::init();
    std::cout << "server inited ~" << std::endl;
    std::cout << std::endl;

    // start server
    server.run();


    return 0;
}