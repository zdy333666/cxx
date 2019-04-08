//
// Created by zdy on 19-4-4.
//

// The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
/*
    This example shows how to run a CNN based face detector using dlib.  The
    example loads a pretrained model and uses it to find faces in images.  The
    CNN model is much more accurate than the HOG based model shown in the
    face_detection_ex.cpp example, but takes much more computational power to
    run, and is meant to be executed on a GPU to attain reasonable speed.  For
    example, on a NVIDIA Titan X GPU, this example program processes images at
    about the same speed as face_detection_ex.cpp.

    Also, users who are just learning about dlib's deep learning API should read
    the dnn_introduction_ex.cpp and dnn_introduction2_ex.cpp examples to learn
    how the API works.  For an introduction to the object detection method you
    should read dnn_mmod_ex.cpp



    TRAINING THE MODEL
        Finally, users interested in how the face detector was trained should
        read the dnn_mmod_ex.cpp example program.  It should be noted that the
        face detector used in this example uses a bigger training dataset and
        larger CNN architecture than what is shown in dnn_mmod_ex.cpp, but
        otherwise training is the same.  If you compare the net_type statements
        in this file and dnn_mmod_ex.cpp you will see that they are very similar
        except that the number of parameters has been increased.

        Additionally, the following training parameters were different during
        training: The following lines in dnn_mmod_ex.cpp were changed from
            mmod_options options(face_boxes_train, 40,40);
            trainer.set_iterations_without_progress_threshold(300);
        to the following when training the model used in this example:
            mmod_options options(face_boxes_train, 80,80);
            trainer.set_iterations_without_progress_threshold(8000);

        Also, the random_cropper was left at its default settings,  So we didn't
        call these functions:
            cropper.set_chip_dims(200, 200);
            cropper.set_min_object_size(40,40);

        The training data used to create the model is also available at
        http://dlib.net/files/data/dlib_face_detection_dataset-2016-09-30.tar.gz
*/

#include <dlib/dnn.h>
#include <dlib/data_io.h>
#include <dlib/image_io.h>

#include "kissrandom.h"
#include "annoylib.h"

#include "utils.cpp"

using namespace std;
using namespace dlib;


// ----------------------------------------------------------------------------------------

template<long num_filters, typename SUBNET> using con5d = con<num_filters, 5, 5, 2, 2, SUBNET>;
template<long num_filters, typename SUBNET> using con5  = con<num_filters, 5, 5, 1, 1, SUBNET>;

template<typename SUBNET> using downsampler  = relu<affine<con5d<32,
        relu<affine<con5d<32, relu<affine<con5d<16, SUBNET>>>>>>>>>;
template<typename SUBNET> using rcon5  = relu<affine<con5<45, SUBNET>>>;

using net_type = loss_mmod<con<1, 9, 9, 1, 1, rcon5<rcon5<rcon5<downsampler<
        input_rgb_image_pyramid<pyramid_down<6>>>>>>>>;

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
using residual = add_prev1<block<N, BN, 1, tag1<SUBNET>>>;

template<template<int, template<typename> class, int, typename> class block, int N,
        template<typename> class BN, typename SUBNET>
using residual_down = add_prev2<avg_pool<2, 2, 2, 2, skip1<tag2<block<N, BN, 2, tag1<SUBNET>>>>>>;

template<int N, template<typename> class BN, int stride, typename SUBNET>
using block  = BN<con<N, 3, 3, 1, 1, relu<BN<con<N, 3, 3, stride, stride, SUBNET>>>>>;

template<int N, typename SUBNET> using ares      = relu<residual<block, N, affine, SUBNET>>;
template<int N, typename SUBNET> using ares_down = relu<residual_down<block, N, affine, SUBNET>>;

template<typename SUBNET> using alevel0 = ares_down<256, SUBNET>;
template<typename SUBNET> using alevel1 = ares<256, ares<256, ares_down<256, SUBNET>>>;
template<typename SUBNET> using alevel2 = ares<128, ares<128, ares_down<128, SUBNET>>>;
template<typename SUBNET> using alevel3 = ares<64, ares<64, ares<64, ares_down<64, SUBNET>>>>;
template<typename SUBNET> using alevel4 = ares<32, ares<32, ares<32, SUBNET>>>;

using anet_type = loss_metric<fc_no_bias<128, avg_pool_everything<
        alevel0<
                alevel1<
                        alevel2<
                                alevel3<
                                        alevel4<
                                                max_pool<3, 3, 2, 2,
                                                        relu<affine<con<
                                                                32, 7, 7, 2, 2,
                                                                input_rgb_image_sized<150>
                                                        >>>>>>>>>>>>;

// ----------------------------------------------------------------------------------------

net_type detector;

// We will also use a face landmarking model to align faces to a standard pose:  (see face_landmark_detection_ex.cpp for an introduction)
shape_predictor predictor;

// And finally we load the DNN responsible for face recognition.
anet_type descriptor;

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
}


int main() try {


    init();

    std::string imgPaths[] = {
            "chendong",
            "liyongliang",
            "shenxiaohui",
            "wangrongfa",
            "wangshanlin",
            "yanpeizong"
    };


    int face_descriptor_dimension = 128;

    //Building the tree
    AnnoyIndex<int, double, Euclidean, Kiss32Random> annoy_index = AnnoyIndex<int, double, Euclidean, Kiss32Random>(face_descriptor_dimension);
    int annoy_index_item_id = 0;


    long t_start, t_end;

    for (std::string imgPath: imgPaths) {

        imgPath = "../data/people_" + imgPath + ".jpg";

        matrix<rgb_pixel> img;
        load_image(img, imgPath);


        t_start = utils::timestamp();

        auto faces = detector(img);

        t_end = utils::timestamp();
        std::cout << "detect face in " << t_end - t_start << " ms" << std::endl;



        // Run the face detector on the image of our action heroes, and for each face extract a
        // copy that has been normalized to 150x150 pixels in size and appropriately rotated
        // and centered.
        std::vector<matrix<rgb_pixel>> extracted_faces;
        for (auto face : faces) {

            double detection_confidence = face.detection_confidence;
            cout << "face detection_confidence: " << detection_confidence << endl;

            if(detection_confidence < 1.0){
                continue;
            }

            auto shape = predictor(img, face.rect);

            matrix<rgb_pixel> face_chip;
            extract_image_chip(img, get_face_chip_details(shape, 150, 0.25), face_chip);

            extracted_faces.push_back(move(face_chip));
        }


        // This call asks the DNN to convert each face image in faces into a 128D vector.
        // In this 128D vector space, images from the same person will be close to each other
        // but vectors from different people will be far apart.  So we can use these vectors to
        // identify if a pair of images are from the same person or from different people.
        std::vector<matrix<float, 0, 1>> face_descriptors = descriptor(extracted_faces);
        int n = face_descriptors.size();



        for (int i = 0; i < n; i++) {

            auto face_descriptor = face_descriptors[i];
//            cout << "face descriptor for one face: " << trans(face_descriptor) << endl;

            std::vector<double> vec(face_descriptor.begin(), face_descriptor.end());

            annoy_index.add_item(annoy_index_item_id++, vec.data());
        }

    }


    std::cout << std::endl;
    std::cout << "Building index num_trees = 2 * num_features ..." << std::endl;

    t_start = utils::timestamp();

    std::cout << "2 * face_descriptor_dimension: " << 2 * face_descriptor_dimension << std::endl;
    annoy_index.build(2 * face_descriptor_dimension);

    t_end = utils::timestamp();
    std::cout << "Builded index in " << t_end - t_start << " ms" << std::endl;


    std::cout << "Saving index ..." << std::endl;
    annoy_index.save("../data/precision.tree");


    std::cout << "get_n_items: " << annoy_index.get_n_items() << std::endl;
    std::cout << "get_n_trees: " << annoy_index.get_n_trees() << std::endl;

    return 0;

} catch (std::exception &e) {
    cout << e.what() << endl;
}


