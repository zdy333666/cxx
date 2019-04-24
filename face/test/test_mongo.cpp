//
// Created by zdy on 19-4-9.
//

#include <cstdint>
#include <iostream>
#include <vector>
#include <limits.h>

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/types.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>
#include <bsoncxx/array/value.hpp>

#include "../json.hpp"


using json = nlohmann::json;


//using bsoncxx::builder::stream::close_array;
//using bsoncxx::builder::stream::close_document;
//using bsoncxx::builder::stream::document;
//using bsoncxx::builder::stream::finalize;
//using bsoncxx::builder::stream::open_array;
//using bsoncxx::builder::stream::open_document;

mongocxx::client client;
mongocxx::database db;


void insert() {

    mongocxx::collection coll = db["test"];


    auto builder = bsoncxx::builder::stream::document();

    bsoncxx::document::value doc = builder
            << "name" << "MongoDB"
            << "type" << "database"
            << "count" << 1
            << "versions" << bsoncxx::builder::stream::open_array
            << "v3.2" << "v3.0" << "v2.6"
            << bsoncxx::builder::stream::close_array
            << "info" << bsoncxx::builder::stream::open_document
            << "x" << 203
            << "y" << 102
            << bsoncxx::builder::stream::close_document
            << bsoncxx::builder::stream::finalize;


    std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;

    bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

    std::cout << "inserted_id:" << result.value().inserted_id().get_oid().value.to_string() << std::endl;

}


/**
        *
        * @param group_id
        * @return
        */
auto group_add(std::string &group_id, std::string &group_name) {

    auto builder = bsoncxx::builder::stream::document{};
    bsoncxx::document::value doc = builder
            << "group_id" << group_id
            << "group_name" << group_name
            << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())                      //DateTime.SpecifyKind(DateTime.Now, DateTimeKind.Utc)
            << bsoncxx::builder::stream::finalize;

    std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;

    mongocxx::collection coll = db["faceset_group"];
    bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

    std::string inserted_id = result.value().inserted_id().get_oid().value.to_string();
    std::cout << "inserted_id:" << inserted_id << std::endl;

    return inserted_id;
}


/**
         *
         * @param start
         * @param length
         * @return
         */
auto getlist(int start, int length) {

    bsoncxx::document::value filter =
            bsoncxx::builder::stream::document() << bsoncxx::builder::stream::finalize;
    std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

    bsoncxx::document::value projection = bsoncxx::builder::stream::document()
            << "_id" << 0
            << bsoncxx::builder::stream::finalize;
    std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;

    mongocxx::options::find options = mongocxx::options::find().skip(start).limit(length).projection( projection.view());

    mongocxx::collection coll = db["faceset_group"];
    mongocxx::cursor cursor = coll.find(filter.view(), options);

    std::vector<bsoncxx::document::value> docs(cursor.begin(), cursor.end());

    std::vector<std::string> result;

    for (bsoncxx::document::value doc : docs) {
        std::string doc_json = bsoncxx::to_json(doc);
        result.push_back(doc_json);

        std::cout << "doc:" << doc_json << std::endl;
    }


    return docs;
}


auto test_add() {

    mongocxx::collection coll = db["test"];

    std::vector<double> descriptors ={
            1.5,
            2.0,
            3.0
    };


    auto builder = bsoncxx::builder::stream::document();

    bsoncxx::document::value doc =
            builder
            << "name" << "MongoDB"
            << "type" << "database"
            << "descriptors" << bsoncxx::builder::stream::open_array
                    << [&](bsoncxx::builder::stream::array_context<> arr) {
                        for (double descriptor :descriptors )
                        {
                            arr << descriptor;
                        }
                    }
            << bsoncxx::builder::stream::close_array
            << "info" << bsoncxx::builder::stream::open_document
            << "x" << 203
            << "y" << 102
            << bsoncxx::builder::stream::close_document
            << bsoncxx::builder::stream::finalize;


    std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;

    bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

    std::cout << "inserted_id:" << result.value().inserted_id().get_oid().value.to_string() << std::endl;

}


std::string test_find() {

    mongocxx::collection coll = db["test"];

//    std::vector<double> descriptors ={
//            1.5,
//            2.0,
//            3.0
//    };


    auto builder = bsoncxx::builder::stream::document();

    bsoncxx::document::value filter =
            builder
//                    << "name" << "MongoDB"
//                    << "type" << "database"
//                    << "descriptors" << bsoncxx::builder::stream::open_array
//                    << [&](bsoncxx::builder::stream::array_context<> arr) {
//                        for (double descriptor :descriptors )
//                        {
//                            arr << descriptor;
//                        }
//                    }
//                    << bsoncxx::builder::stream::close_array
//                    << "info" << bsoncxx::builder::stream::open_document
//                    << "x" << 203
//                    << "y" << 102
//                    << bsoncxx::builder::stream::close_document
                    << bsoncxx::builder::stream::finalize;


    std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

    bsoncxx::stdx::optional<bsoncxx::document::value> result = coll.find_one(filter.view());

    return  bsoncxx::to_json(result.value());

}

int main() {

//    mongocxx::instance instance{}; // This should be done only once.
//    mongocxx::uri uri("mongodb://192.168.8.179:27017");
//
//    client = mongocxx::client(uri);
//    db = client["face"];
////
////    std::string group_id = bsoncxx::oid().to_string();
////    std::string group_name = "group2";
////
//////    group_add(group_id, group_name);
////
////
////
//    getlist(0, 100);

//
//    std::string json_str = test_find();
//    std::cout << "json_str:" << json_str << std::endl;
//
//    json doc_json = json::parse(json_str);
//
//    std::vector<double> descriptors = doc_json["descriptors"];
//    std::cout << "descriptors size:" << descriptors.size() << std::endl;
//
//    for(double descriptor : descriptors){
//        std::cout << "descriptors:" << descriptor << std::endl;
//    }


//    test();

//    double arr[3] = {1.0, 2.0, 3.0};
//    double *baa = arr;

//
//    std::cout << "INT_MAX:" << bsoncxx::to_json() << std::endl;

int arr[] ={1,2,3};

int* bbb =arr;

for(int n:arr){
    std::cout << "n:" << n << std::endl;
}

    return 0;
}


