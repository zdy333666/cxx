//
// Created by zdy on 19-5-8.
//

#include <iostream>
#include<sys/time.h>
#include <b64/encode.h>
#include <b64/decode.h>

#include "opencv2/opencv.hpp"
//#include <opencv2/features2d.hpp>
//#include <opencv2/nonfree/nonfree.hpp> // use this if you want to use SIFT or SURF

#include "../common/json.hpp"
#include "../common/mongoose.h"
#include "../common/mongoose.c"

using namespace std;
using json = nlohmann::json;


cv::Mat frame;


long timestamp() {

    struct timeval tv;
    gettimeofday(&tv, NULL);

    return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}


/* 调用libb64对数据进行base64编码 * input为nullptr或size为0抛出std::invalid_argument异常 * */
std::string encode(const char *input, size_t size) {

    std::vector<char> buffer(size << 1);
    base64::encoder ec;
    base64_init_encodestate(&ec._state);

    // count为base64编码后的数据长度
    auto count = ec.encode(reinterpret_cast<const char *>(input), int(size), buffer.data());
    count += ec.encode_end(buffer.data() + count);
    assert(count <= buffer.size());

    return std::string(buffer.data(), buffer.data() + count);
}

/* 调用libb64对base64编码的字符串进行解码，返回解码后的二进制数组 * input为空抛出std::invalid_argument异常 * */
std::string decode(const std::string &input) {

    std::vector<char> buffer(input.size());
    base64::decoder dc;
    base64_init_decodestate(&dc._state);

    // count为base64解码后的数据长度
    auto count = dc.decode(input.data(), int(input.size()), reinterpret_cast<char *>(buffer.data()));
    assert(count <= buffer.size());

    return std::string(buffer.data(), buffer.data() + count);
}


/* RESTful server host and request URI */
static const char *s_url = "http://192.168.8.120:8080/face/v1/search";

static int s_exit_flag = 0;

static void ev_handler(struct mg_connection *nc, int ev, void *ev_data) {
    struct http_message *hm = (struct http_message *) ev_data;
    int connect_status;

    switch (ev) {
        case MG_EV_CONNECT:
            connect_status = *(int *) ev_data;
            if (connect_status != 0) {
                printf("Error connecting to %s: %s\n", s_url, strerror(connect_status));
                s_exit_flag = 1;
            }
            break;
        case MG_EV_HTTP_REPLY:
//            printf("Got reply:\n%.*s\n", (int) hm->body.len, hm->body.p);
            nc->flags |= MG_F_SEND_AND_CLOSE;
            s_exit_flag = 1;

            try {
                json body = json::parse(hm->body.p, hm->body.p + hm->body.len);
                if (body["error_code"] == 0) {
                    json user_list = body["user_list"];

                    std::cout << "user_list： " << user_list << std::endl;

//                        for (json face : face_list) {
//                            json location = face["location"];
//                            int left = cvRound((double) location["left"]);
//                            int top = cvRound((double) location["top"]);
//                            int width = cvRound((double) location["width"]);
//                            int height = cvRound((double) location["height"]);
//
//                            std::cout << "left： " << left << std::endl;
//                            std::cout << "top： " << top << std::endl;
//                            std::cout << "width： " << width << std::endl;
//                            std::cout << "height： " << height << std::endl;
//
//                            //Rect(int a,int b,int c,int d)a,b为矩形的左上角坐标,c,d为矩形的长和宽
////                            cv::rectangle(frame, cv::Rect(left, top, height, width), cv::Scalar(0, 255, 0), 1, 1, 0);
//                            cv::rectangle(frame, cvPoint(left, top), cvPoint(left + width, top + height),
//                                          cv::Scalar(0, 255, 0), 1, 1, 0);
//                        }
                }
            } catch (std::exception &e) {
                std::cout << e.what() << std::endl;
            }

            break;
        case MG_EV_CLOSE:
            if (s_exit_flag == 0) {
                printf("Server closed connection\n");
                s_exit_flag = 1;
            };
            break;
        default:
            break;
    }
}


int main() {


//    const char *text = "Matt Damon";
//    std::string encode_str = encode(text, strlen(text));
//    cout << "encode: " << encode_str << endl;
//
//    std::string decode_str = decode(encode_str);
//    cout << "decode: " << decode_str.data() << endl;
//
//    if (strlen(text) > 0) {
//        return 0;
//    }

    // 声明特征提取器与描述子提取器
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;

    detector = cv::ORB::create();
    descriptor = detector;

//     如果使用 sift, surf ，之前要初始化nonfree模块
//     cv::initModule_nonfree();
//     detector = cv::BRISK::create();
//     descriptor =detector;



    cv::namedWindow("video");

    cv::VideoCapture cap = cv::VideoCapture(0);

    while (cap.isOpened()) {

        cap >> frame;
        cout << "frame size: " << frame.cols << "," << frame.rows << endl;


        cv::Mat rgb;
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);


        long t_start, t_end;

//        //关键点
//
//        t_start = timestamp();
//
//        vector<cv::KeyPoint> kp;
//        detector->detect(rgb, kp);
//
//        t_end = timestamp();
//        cout << "Key points of frame: " << kp.size() << " in " << t_end - t_start << " ms" << endl;
//
//
//        // 计算描述子
//
//        t_start = timestamp();
//
//        cv::Mat desp;
//        descriptor->compute(rgb, kp, desp);
//
//        t_end = timestamp();
//        cout << "descriptors of frame: " << desp.size() << " in " << t_end - t_start << " ms" << endl;





        t_start = timestamp();

        std::string img_file_path = "../temp/frame.jpg";

        // 缓存图片帧到
        cv::imwrite(img_file_path, frame);

        std::ifstream img_file(img_file_path, ios::in | ios::binary);
        if (!img_file.is_open()) {
            std::cout << "can not access img file:" << img_file_path << std::endl;
            exit(1);
        }

        img_file.seekg(0, img_file.end);   //追溯到流的尾部
        int size = img_file.tellg();  //获取流的长度
        img_file.seekg(0, img_file.beg);  //回到流的头部

        char *raw_image = new char[size];   //用来暂存内容的数组
        img_file.read(raw_image, size);    //read函数
        img_file.close();

        t_end = timestamp();
        std::cout << "save image in " << t_end - t_start << " ms" << std::endl;


        t_start = timestamp();

        std::string base64_frame = encode(raw_image, size);
        delete[] raw_image;

        t_end = timestamp();
        cout << "base64 encode of frame in " << t_end - t_start << " ms" << endl;

        cout << "raw frame length : " << size << endl;
        cout << "base64 frame length: " << base64_frame.length() << endl;


        t_start = timestamp();

        json post_data = json::object();
        post_data["image"] = base64_frame;
        post_data["image_type"] = "BASE64";
        post_data["group_id_list"] = "5cad68f6bab4af7a4908e5c2";
        post_data["max_user_num"] = 1;


        struct mg_mgr mgr;
//        struct mg_connection *nc;

        mg_mgr_init(&mgr, NULL);
        mg_connect_http(&mgr, ev_handler, s_url, "Content-Type: application/json\r\n", post_data.dump().data());

        printf("Starting RESTful client against %s\n", s_url);
        while (s_exit_flag == 0) {
            mg_mgr_poll(&mgr, 100);
        }

        mg_mgr_free(&mgr);
        s_exit_flag = 0;


        t_end = timestamp();
        cout << "face detect of frame in " << t_end - t_start << " ms" << endl;


        cv::imshow("video", frame);
        if (cv::waitKey(1) == 'q') {
            break;
        }

        cout << endl;

//        if (frame.total() > 0) {
//            return 0;
//        }
    }


    return 0;
}