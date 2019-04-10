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

    int nn = 3;
    flann::Matrix<float> dataset;
    flann::Matrix<float> query;

    flann::load_from_file(dataset, "../data/dataset.hdf5", "dataset");
    flann::load_from_file(query, "../data/dataset.hdf5", "query");


    std::cout << "dataset.rows:" << dataset.rows << std::endl;
    std::cout << "dataset.cols:" << dataset.cols << std::endl;


    std::cout << "query.rows:" << query.rows << std::endl;
    std::cout << "query.cols:" << query.cols << std::endl;

    flann::Matrix<int> indices(new int[query.rows * nn], query.rows, nn);
    flann::Matrix<float> dists(new float[query.rows * nn], query.rows, nn);


// construct an randomized kd-tree index using 4 kd-trees
    flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();


// do a knn search, using 128 checks
    index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
    flann::save_to_file(indices, "../data/result.hdf5", "result");


    std::cout << "indices:" << indices.rows << std::endl;
    std::cout << "dists:" << dists.rows << std::endl;


    delete[] dataset.ptr();
    delete[] query.ptr();
    delete[] indices.ptr();
    delete[] dists.ptr();

    return 0;
}