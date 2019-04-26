//
// Created by zdy on 19-4-6.
//

#include <boost/container/stable_vector.hpp>

#include <dlib/dnn.h>
#include <dlib/data_io.h>
#include <dlib/image_io.h>

#include <bsoncxx/oid.hpp>
#include "utils.cpp"

#include "kissrandom.h"
#include "annoylib.h"

//#include <flann/flann.hpp>

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
        Location* location;
        boost::container::stable_vector<Point*>* landmarks;
        boost::container::stable_vector<double>* descriptors;

    public:
        FaceInfo() {

        };

        ~FaceInfo() {

//            delete landmarks;
//            landmarks = NULL;
//
//            delete descriptors;
//            descriptors = NULL;
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

        void set_location(Location* location) {
            this->location = location;
        }

        Location* get_location() {
            return this->location;
        }

        void set_landmarks(boost::container::stable_vector<Point*>* landmarks) {
            this->landmarks = landmarks;
        }

        boost::container::stable_vector<Point*>* get_landmarks() {
            return this->landmarks;
        }

        void set_descriptors(boost::container::stable_vector<double>* descriptors) {
            this->descriptors = descriptors;
        }

        boost::container::stable_vector<double>* get_descriptors() {
            return this->descriptors;
        }

    };


    // ----------------------------------------------------------------------------------------

    net_type detector;

    // We will also use a face landmarking model to align faces to a standard pose:  (see face_landmark_detection_ex.cpp for an introduction)
    shape_predictor predictor;

    // And finally we load the DNN responsible for face recognition.
    anet_type descriptor;


    // group_id link to  index_user_map
    std::map<std::string, std::map<int, std::string>* >* group_user_map = new std::map<std::string, std::map<int, std::string>* >();

    // group_id link to face feature index
    std::map<std::string, AnnoyIndex<int, double, Euclidean, Kiss32Random>*>* group_index_map = new std::map<std::string, AnnoyIndex<int, double, Euclidean, Kiss32Random>*>();

    //    std::map<std::string, flann::Index<flann::L2<double> > > group_index_map;

    /**
     * build group feature index
     *
     * @param group_id
     * @param user_ids
     * @param feature_list
     */
    void build_group_index(std::string& group_id, boost::container::stable_vector<std::string> user_ids, boost::container::stable_vector<std::vector<double>>& feature_list){

        std::cout << "build face index of group_id:" << group_id << std::endl;

//        flann::Matrix<double> dataset(new double[feature_list.size()*128], feature_list.size(), 128);
//
//        for (int n = 0; n < feature_list.size(); n++) {
//            std::vector<double> features = feature_list[n];
//            for (int k = 0; k < features.size(); k++) {
//                dataset[n][k] = n;
//            }
//        }
//
//        flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(4));
//        index.buildIndex();

        AnnoyIndex<int, double, Euclidean, Kiss32Random>* index = new AnnoyIndex<int, double, Euclidean, Kiss32Random>(128);

        for (int n = 0; n < feature_list.size(); n++) {
            std::vector<double> features = feature_list[n];

            double* features_array = new double[features.size()];
            for (int k = 0; k < features.size(); k++) {
                features_array[k] = features[k];
            }

            index->add_item(n, features_array);

            delete[] features_array;
            features_array = NULL;

            features.clear();
            std::vector<double>().swap(features);
        }

        index->build(4);

        AnnoyIndex<int, double, Euclidean, Kiss32Random>* old_index = (*group_index_map)[group_id];

        if(old_index != NULL){

            std::cout << "old_index:" << old_index << std::endl;

            delete old_index;
            old_index = NULL;
        }

        (*group_index_map)[group_id] = move(index);

        //---------------------------------------------------------------------------

        std::map<int, std::string>* index_user_map = new std::map<int, std::string>();
        for(int m = 0; m < user_ids.size(); m++){
            (*index_user_map)[m] = user_ids[m];
        }

        std::map<int, std::string>* old_index_user_map = (*group_user_map)[group_id];

        if(old_index_user_map != NULL){

            std::cout << "old_index_user_map:" << old_index_user_map << std::endl;

            delete old_index_user_map;
            old_index_user_map = NULL;
        }

        (*group_user_map)[group_id] = move(index_user_map);
    }


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


//        std::string annoy_index_path = "../data/faceset_test.tree";
//        annoy_index.load(annoy_index_path.c_str());
//        std::cout << "loaded annoy index file:" << annoy_index_path << std::endl;
//
//        std::cout << "annoy_index_n_items: " << annoy_index.get_n_items() << std::endl;
//        std::cout << "annoy_index_n_trees: " << annoy_index.get_n_trees() << std::endl;
    }


    /**
    *
    * @param img_str
    * @param img_id
    * @return
    */
     dlib::matrix<dlib::rgb_pixel>& local_load_image(dlib::matrix<dlib::rgb_pixel>& img, const std::string &img_str, const std::string &img_id) {

        long t_start, t_end;
        t_start = utils::timestamp();

        std::string img_file_path = "../temp/" + img_id + ".jpg";

        std::ofstream img_file;
        img_file.open(img_file_path);

        if (!img_file.is_open()) {
            std::cout << "can not access img file:" << img_file_path << std::endl;
            exit(1);
        }

        img_file.write(img_str.data(), img_str.size());
        img_file.close();


        dlib::load_image(img, img_file_path);

        std::remove(img_file_path.data());

        t_end = utils::timestamp();
        std::cout << "save image in " << t_end - t_start << " ms" << std::endl;

        return img;
    }


    /**
     * 
     * @param img 
     * @return 
     */
    std::vector<dlib::mmod_rect>& local_get_faces(std::vector<dlib::mmod_rect>& faces, const dlib::matrix<dlib::rgb_pixel>& img) {

        long t_start, t_end;
        t_start = utils::timestamp();

        faces = detector(img);

        t_end = utils::timestamp();
        std::cout << "detect face in " << t_end - t_start << " ms" << std::endl;

        return faces;
    }

    /**
     * 
     */
std::vector<matrix < float, 0, 1>>& local_get_descriptors(std::vector<matrix < float, 0, 1>>& descriptors, const boost::container::stable_vector<matrix<rgb_pixel>>& faces) {

    long t_start, t_end;
    t_start = utils::timestamp();

    // This call asks the DNN to convert each face image in faces into a 128D vector.
    // In this 128D vector space, images from the same person will be close to each other
    // but vectors from different people will be far apart.  So we can use these vectors to
    // identify if a pair of images are from the same person or from different people.

    descriptors = descriptor(faces);

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
std::vector<int>* local_ann_search( std::vector<int>* closest, const double *descriptor, int result_n, const std::string &group_id) {

    long t_start, t_end;
    t_start = utils::timestamp();

    // group_id link to face feature index
    AnnoyIndex<int, double, Euclidean, Kiss32Random>* index = (*group_index_map)[group_id];

    if(index == NULL){
        return closest;
    }


    // search face from annoy index tree
    int search_k = -1;
    std::vector<int> tmp_closest = std::vector<int>();
    std::vector<double> distances = std::vector<double>();

    index->get_nns_by_vector(descriptor, result_n, search_k, &tmp_closest, &distances);

    t_end = utils::timestamp();
    std::cout << "face search with group_id:" << group_id << " in " << t_end - t_start << " ms" << std::endl;

    for (int i = 0; i < tmp_closest.size(); i++) {
        int id = tmp_closest[i];
        double distance = distances[i];

        // set the max distance
        if(distance > 0.2){
            continue;
        }

        closest->push_back(id);

        std::cout << "search result -- id:" << id << "  distance:" << distance << std::endl;
    }

    return closest;
}


/**
 *
 * @param userInfos
 * @param descriptors
 * @param result_n
 * @param group_ids
 * @return
 */
boost::container::stable_vector<UserInfo*>* user_search(boost::container::stable_vector<UserInfo*>* userInfos, boost::container::stable_vector<double>* descriptors, int result_n, std::vector <string>* group_ids) {

    double descriptors_array[descriptors->size()];

    for(int n = 0; n < descriptors->size(); n++){
        descriptors_array[n] = (*descriptors)[n];
    }

    std::vector<int> closest = std::vector<int>();

    for(std::string group_id : (*group_ids)) {

        local_ann_search(&closest, descriptors_array, result_n, group_id);

        for (int id : closest) {
            UserInfo* userInfo = new UserInfo();
            userInfo->group_id = group_id;
            userInfo->user_id = (*(*group_user_map)[group_id])[id];
            userInfo->score = 0.0;

            userInfos->push_back(userInfo);
        }

        result_n = result_n - closest.size();
        closest.clear();

        if(result_n == 0){
            break;
        }
    }

    return userInfos;
}


/**
*
* @param img_str
* @return
*/
boost::container::stable_vector<FaceInfo*>* face_detect(boost::container::stable_vector<FaceInfo*>* faceInfos, const dlib::matrix<dlib::rgb_pixel> &img, int max_face_num) {

    std::vector<dlib::mmod_rect> faces;
    local_get_faces(faces, img);

    // Run the face detector on the image of our action heroes, and for each face extract a
    // copy that has been normalized to 150x150 pixels in size and appropriately rotated
    // and centered.
    boost::container::stable_vector<matrix<rgb_pixel>> *extracted_faces = new boost::container::stable_vector<matrix<rgb_pixel>>();
    boost::container::stable_vector<int> *extracted_faces_index = new boost::container::stable_vector<int>();

    cout << "faces.size: " << faces.size() << endl;

    for (int i = 0; i < faces.size(); i++) {

        dlib::mmod_rect face = faces[i];

        double detection_confidence = face.detection_confidence;
        cout << "face " << i << " detection_confidence: " << detection_confidence << endl;

        //set face detection confidence thresholdv
        if (detection_confidence < 0.95) {
            continue;
        }


        if (extracted_faces->size() == max_face_num) {
            break;
        }

        auto shape = predictor(img, face.rect);

        matrix<rgb_pixel> face_chip;
        extract_image_chip(img, get_face_chip_details(shape, 150, 0.25), face_chip);

        extracted_faces->push_back(move(face_chip));
        extracted_faces_index->push_back(i);

    }

    std::vector<matrix<float, 0, 1>> face_descriptors;
    local_get_descriptors(face_descriptors, *extracted_faces);

//    std::cout << "extracted_faces clear ..." << std::endl;
    delete extracted_faces;
    extracted_faces = NULL;
//    std::cout << "extracted_faces cleared" << std::endl;


    for (int i = 0; i < face_descriptors.size(); i++) {

        int face_index = (*extracted_faces_index)[i];
        std::cout << "face_index:" << face_index << std::endl;

        auto face = faces[face_index];
        auto face_descriptor = face_descriptors[i];

        auto shape = predictor(img, face.rect);
//        std::cout << "shape rect:" << shape.get_rect() << std::endl;
//        std::cout << "shape size:" << shape.num_parts() << std::endl;

        std::string face_token = bsoncxx::oid().to_string();
        double face_probability = face.detection_confidence;
        Location *location = new Location();
        boost::container::stable_vector<Point *> *landmarks = new boost::container::stable_vector<Point *>();
        boost::container::stable_vector<double> *descriptors = new boost::container::stable_vector<double>(
                face_descriptor.begin(), face_descriptor.end());


        location->left = face.rect.left();
        location->top = face.rect.top();
        location->width = face.rect.width();
        location->height = face.rect.height();
        location->rotation = 0;


//        std::cout << "shape num_parts:" << shape.num_parts() << std::endl;

        for (int idx = 0; idx < shape.num_parts(); idx++) {
            auto part = shape.part(idx);

            Point *point = new Point();
            point->x = part.x();
            point->y = part.y();

            landmarks->push_back(point);
        }

//        std::cout << "create faceInfo " << i << std::endl;

        FaceInfo *faceInfo = new FaceInfo();
        faceInfo->set_face_token(face_token);
        faceInfo->set_face_probability(face_probability);
        faceInfo->set_location(location);
        faceInfo->set_landmarks(landmarks);
        faceInfo->set_descriptors(descriptors);

//        std::cout << "push faceInfo " << i << std::endl;

        faceInfos->push_back(faceInfo);

//        std::cout << "pushed faceInfo " << i << std::endl;
    }

//    std::cout << "face_descriptors clear ..." << std::endl;
    face_descriptors.clear();
    std::vector<matrix<float, 0, 1>>().swap(face_descriptors);
//    std::cout << "face_descriptors cleared" << std::endl;

//    std::cout << "extracted_faces_index clear ..." << std::endl;
    delete extracted_faces_index;
    extracted_faces_index = NULL;
//    std::cout << "extracted_faces_index cleared" << std::endl;

    return faceInfos;
}

}
