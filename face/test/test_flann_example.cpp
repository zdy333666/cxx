//
// Created by zdy on 19-4-9.
//

/**
http://www.cs.ubc.ca/research/flann/uploads/FLANN/datasets/dataset.hdf5
http://www.cs.ubc.ca/research/flann/uploads/FLANN/datasets/dataset.dat
http://www.cs.ubc.ca/research/flann/uploads/FLANN/datasets/testset.dat
 */

// file flann_example.cpp
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>
#include <stdio.h>

int main(int argc, char **argv) {

    try {

        int nn = 3;

        int rows = 10000;
        int cols = 128;

        flann::Matrix<float> dataset(new float[rows * cols], rows, cols);
        flann::Matrix<float> query(new float[1 * cols], 1, cols);

//    flann::load_from_file(dataset, "../data/dataset.hdf5", "dataset");
//    flann::load_from_file(query, "../data/dataset.hdf5", "query");
//


        for (int n = 0; n < rows; n++) {
            for (int k = 0; k < cols; k++) {
                dataset[n][k] = (float) n;
            }
        }

        std::cout << "dataset.rows:" << dataset.rows << std::endl;
        std::cout << "dataset.cols:" << dataset.cols << std::endl;



        std::string index_file_path = "../data/flann_index.dat";


        // construct an randomized kd-tree index using 4 kd-trees
//
//        flann::Index<flann::L2<float> > pre_index(dataset, flann::KDTreeIndexParams(4));
//        pre_index.buildIndex();
//        pre_index.save(index_file_path);
//
//        std::cout << "pre_index saved" << std::endl << std::endl;



        flann::IndexParams params;
        params["algorithm"] = flann::FLANN_INDEX_SAVED;
        params["filename"] = index_file_path;

        flann::KDTreeIndex<flann::L2<float> > index (dataset, params);


        std::cout << "index size:" << index.size() << std::endl;
        std::cout << "index veclen:" << index.veclen() << std::endl << std::endl;


        flann::Matrix<float> point(new float[1 * cols], 1, cols);
        for (int k = 0; k < cols; k++) {
            point[0][k] = 7;
        }

        index.addPoints(point);

        std::cout << "index size 2:" << index.size() << std::endl;
        std::cout << "index veclen 2:" << index.veclen() << std::endl << std::endl;

//        index.removePoint(0);
//        index.buildIndex();
//
//        std::cout << "index size 3:" << index.size() << std::endl << std::endl;



        for (int k = 0; k < cols; k++) {
            query[0][k] = 7;
        }

        std::cout << "query.rows:" << query.rows << std::endl;
        std::cout << "query.cols:" << query.cols << std::endl;


        flann::Matrix<int> indices(new int[query.rows * nn], query.rows, nn);
        flann::Matrix<float> dists(new float[query.rows * nn], query.rows, nn);


        // do a knn search, using 128 checks
        index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
        flann::save_to_file(indices, "../data/result.hdf5", "result");


        std::cout << "indices rows:" << indices.rows << std::endl;
        std::cout << "dists rows:" << dists.rows << std::endl << std::endl;

        for (int n = 0; n < indices.rows; n++) {

            std::cout << "indices:";
            for (int k = 0; k < indices.cols; k++) {
                std::cout << "  " << indices[n][k];
            }
            std::cout << std::endl;


            std::cout << "dists:";
            for (int k = 0; k < dists.cols; k++) {
                std::cout << "  " << dists[n][k];
            }
            std::cout << std::endl;


            std::cout << std::endl;
        }




        delete[] dataset.ptr();
        delete[] query.ptr();



        delete[] indices.ptr();
        delete[] dists.ptr();

    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
    }

        return 0;
    }