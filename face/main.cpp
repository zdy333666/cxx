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

        json result_json = face::face_detect(image);


        // builder response body

        json result;
        result["error_code"] = 0;
        result["error_msg"] = "SUCCESS";
        result["log_id"] = bsoncxx::oid().to_string();
        result["timestamp"] = utils::timestamp();
        result["cached"] = 0;
        result["result"] = result_json;
//                {
//                {"face_num",  2},
//                {"face_list", {
//                                      {
//                                              {"face_token", bsoncxx::oid().to_string()},
//                                              {"location", {
//                                                                   {"left", 167.24},
//                                                                   {"top", 140.29},
//                                                                   {"width", 208},
//                                                                   {"height", 191},
//                                                                   {"rotation", 1}
//                                                           }
//                                              },
//                                              {"face_probability", 0.96}
//                                      },
//                                      {
//                                              {"face_token", bsoncxx::oid().to_string()},
//                                              {"location", {
//                                                                   {"left", 167.24},
//                                                                   {"top", 140.29},
//                                                                   {"width", 208},
//                                                                   {"height", 191},
//                                                                   {"rotation", 1}
//                                                           }
//                                              },
//                                              {"face_probability", 0.95}
//                                      }
//                              }
//                }
//        };

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

        std::cout << "param:" << param << std::endl;

        std::string image = param["image"].get<std::string>();
        std::string image_type = param["image_type"].get<std::string>();
        std::string group_id_list = param["group_id_list"].get<std::string>();
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

        // do search ...

        json result_json = face::face_search(image, group_id_list, max_face_num);


        // builder response body

        json result;
        result["error_code"] = 0;
        result["error_msg"] = "SUCCESS";
        result["log_id"] = std::rand();
        result["timestamp"] = utils::timestamp();
        result["cached"] = 0;
        result["result"] = result_json;
//                    {
//                    {"user_list", {
//                                          {
//                                                  {"group_id", "faceset_test"},
//                                                  {"user_id", "people_chendong_1"},
//                                                  {"user_info", ""},
//                                                  {"score", 100}
//                                          }, {
//                                                     {"group_id", "faceset_test"},
//                                                     {"user_id", "people_chendong_2"},
//                                                     {"user_info", ""},
//                                                     {"score", 98}
//                                             }
//                                  }
//                    }
//            };

        res.add_header("Content-Type", "application/json");
        res.set_status_and_content(status_type::ok, result.dump());

    }, log_t{});


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