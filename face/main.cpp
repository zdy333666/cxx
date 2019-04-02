//
// Created by zdy on 19-4-1.
//

/**
 *  https://github.com/qicosmos/cinatra
 */

#include "http_server.hpp"
#include "nlohmann_json.hpp"

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include <bsoncxx/oid.hpp>


using namespace cinatra;
using json = nlohmann::json;

using namespace boost::archive::iterators;


long getCurrentTime() {

    struct timeval tv;
    gettimeofday(&tv, NULL);

    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

bool Base64Encode(const std::string &input, std::string *output) {

    typedef base64_from_binary<transform_width<std::string::const_iterator, 6, 8>> Base64EncodeIterator;
    std::stringstream result;

    try {
        copy(Base64EncodeIterator(input.begin()), Base64EncodeIterator(input.end()),
             std::ostream_iterator<char>(result));
    } catch (...) {
        return false;
    }

    size_t equal_count = (3 - input.length() % 3) % 3;
    for (size_t i = 0; i < equal_count; i++) {
        result.put('=');
    }

    *output = result.str();

    return output->empty() == false;
}


bool Base64Decode(const std::string &input, std::string *output) {

    typedef transform_width<binary_from_base64<std::string::const_iterator>, 8, 6> Base64DecodeIterator;
    std::stringstream result;

    try {
        copy(Base64DecodeIterator(input.begin()), Base64DecodeIterator(input.end()),
             std::ostream_iterator<char>(result));
    } catch (...) {
        return false;
    }

    *output = result.str();

    return output->empty() == false;
}


//日志切面
struct log_t {
    bool before(request &req, response &res) {
        std::cout << "before log" << std::endl;
        return true;
    }

    bool after(request &req, response &res) {
        std::cout << "after log" << std::endl;
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
        {

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;


            std::string image = param["image"].get<std::string>();
            std::string image_type = param["image_type"].get<std::string>();
            std::string face_field = param["face_field"].get<std::string>();


            // decode base64 string
            if ("BASE64" == image_type) {
                std::string output_str;
                Base64Decode(image, &output_str);

                std::cout << "base64 size:" << image.size() << std::endl;
                std::cout << "decode size:" << output_str.size() << std::endl;

                image = output_str;
            }

            // do detect ...

        }


        // builder response body
        {

            json result;
            result["error_code"] = 0;
            result["error_msg"] = "SUCCESS";
            result["log_id"] = std::rand();
            result["timestamp"] = getCurrentTime();
            result["cached"] = 0;
            result["result"] = {
                    {"face_num",  2},
                    {"face_list", {
                                          {
                                                  {"face_token", bsoncxx::oid().to_string()},
                                                  {"location", {
                                                                       {"left", 167.24},
                                                                       {"top", 140.29},
                                                                       {"width", 208},
                                                                       {"height", 191},
                                                                       {"rotation", 1}
                                                               }
                                                  },
                                                  {"face_probability", 0.96}
                                          },
                                          {
                                                  {"face_token", bsoncxx::oid().to_string()},
                                                  {"location", {
                                                                       {"left", 167.24},
                                                                       {"top", 140.29},
                                                                       {"width", 208},
                                                                       {"height", 191},
                                                                       {"rotation", 1}
                                                               }
                                                  },
                                                  {"face_probability", 0.95}
                                          }
                                  }
                    }
            };

            res.add_header("Content-Type", "application/json");
            res.set_status_and_content(status_type::ok, result.dump());

        }

    }, log_t{});




    /**
     * face search
     */
    server.set_http_handler<GET, POST>("/face/v1/search", [](request &req, response &res) {

        // process request body
        {

            std::string_view body = req.body();
            json param = json::parse(body);

            std::cout << "param:" << param << std::endl;


            std::string image = param["image"].get<std::string>();
            std::string image_type = param["image_type"].get<std::string>();
            std::string group_id_list = param["group_id_list"].get<std::string>();

            // decode base64 string
            if ("BASE64" == image_type) {
                std::string output_str;
                Base64Decode(image, &output_str);

                std::cout << "base64 size:" << image.size() << std::endl;
                std::cout << "decode size:" << output_str.size() << std::endl;

                image = output_str;
            }

            // do search ...

        }


        // builder response body
        {

            json result;
            result["error_code"] = 0;
            result["error_msg"] = "SUCCESS";
            result["log_id"] = std::rand();
            result["timestamp"] = getCurrentTime();
            result["cached"] = 0;
            result["result"] = {
                    {"user_list", {
                                          {
                                                  {"group_id", "faceset_test"},
                                                  {"user_id", "people_chendong_1"},
                                                  {"user_info", ""},
                                                  {"score", 100}
                                          }, {
                                                     {"group_id", "faceset_test"},
                                                     {"user_id", "people_chendong_2"},
                                                     {"user_info", ""},
                                                     {"score", 98}
                                             }
                                  }
                    }
            };

            res.add_header("Content-Type", "application/json");
            res.set_status_and_content(status_type::ok, result.dump());

        }

    }, log_t{});


    /**
     *  close server
     */
    server.set_http_handler<GET, POST>("/close", [&](request &req, response &res) {
        res.set_status_and_content(status_type::ok, "will close");

        server.stop();
    });


    // start server
    server.run();


    return 0;
}