//
// Created by zdy on 19-4-4.
//

#include <iostream>
#include <random>
#include <chrono>
#include <iomanip>

#include "kissrandom.h"
#include "annoylib.h"

int main() {


    int f = 68;
    int n = 1000;


    std::chrono::high_resolution_clock::time_point t_start, t_end;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 1.0);


    //******************************************************
    //Building the tree
    AnnoyIndex<int, double, Euclidean, Kiss32Random> t = AnnoyIndex<int, double, Euclidean, Kiss32Random>(f);

    std::cout << "Building index ... be patient !!" << std::endl;
    std::cout << "\"Trees that are slow to grow bear the best fruit\" (Moliere)" << std::endl;


    for (int i = 0; i < n; ++i) {
        double *vec = (double *) malloc(f * sizeof(double));

        for (int z = 0; z < f; ++z) {
            vec[z] = (double) i;  //(distribution(generator));
        }

        t.add_item(i, vec);

        std::cout << "Loading objects ...\t object: " << i + 1 << "\tProgress:" << std::fixed << std::setprecision(2)
                  << (double) i / (double) (n + 1) * 100 << "%\r";

    }


    std::cout << std::endl;
    std::cout << "Building index num_trees = 2 * num_features ...";

    t_start = std::chrono::high_resolution_clock::now();
    t.build(2 * f);

    t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(t_end - t_start).count();

    std::cout << " Done in " << duration << " secs." << std::endl;


    std::cout << "Saving index ...";
    t.save("../data/precision.tree");


    std::cout << "get_n_items: " << t.get_n_items() << std::endl;
    std::cout << "get_n_trees: " << t.get_n_trees() << std::endl;



    //-------------------------------------------------------------------------

    double *vec = (double *) malloc(f * sizeof(double));

    for (int z = 0; z < f; ++z) {
        vec[z] = (double) 1000;
    }


    int j = 1000;  //rand() % n;
    int K = 1;
    std::vector<int> closest;

//    std::cout << "finding nbs for " << j << std::endl;

    // getting the K closest
//    t.get_nns_by_item(j, K, n, &closest, nullptr);


//    //全部输出
//    std::cout << "-- closest: ";
//
//    for (vector<int>::iterator it = closest.begin(); it != closest.end(); it++) {
//        std::cout << *it << " ";
//    }
//
//    std::cout << "--" << std::endl;


//    double *result = (double *) malloc(f * sizeof(double));


    std::vector<double> distances;

    t.get_nns_by_vector(vec, K, n, &closest, &distances);

    //全部输出
    std::cout << "-- closest: ";
    for (vector<int>::iterator it = closest.begin(); it != closest.end(); it++) {
        std::cout << *it << " ";
    }
    std::cout << "--" << std::endl;


    std::cout << "-- distances: ";
    for (vector<double>::iterator d_it = distances.begin(); d_it != distances.end(); d_it++) {
        std::cout << *d_it << " ";
    }
    std::cout << "--" << std::endl;


    closest.clear();
    vector<int>().swap(closest);


    std::cout << " Done" << std::endl;


    return 0;
}





















//for (
//int i = 0;
//i< 1000; i++){
//   i v
//for (
//int j = 0;
//j<f;
//j++){
//int v = rand();
//}
//t.
//add_item(i, v
//);
//}
//
//t.build(10); //10 trees
//t.save('test.ann');
//
//
//u = AnnoyIndex(f)
//u.load('test.ann')
//#
//
//super fast, will
//just mmap
//the file
//std::cout(u
//.get_nns_by_item(0, 1000))
#

//will find
//the 1000
//nearest neighbors