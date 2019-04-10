//
// Created by zdy on 19-4-9.
//

#include <cstdint>
#include <iostream>
#include <vector>
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>

#include <bsoncxx/builder/stream/document.hpp>

//using bsoncxx::builder::stream::close_array;
//using bsoncxx::builder::stream::close_document;
//using bsoncxx::builder::stream::document;
//using bsoncxx::builder::stream::finalize;
//using bsoncxx::builder::stream::open_array;
//using bsoncxx::builder::stream::open_document;

mongocxx::client client;
mongocxx::database db;


auto insert() {

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

    bsoncxx::document::value filter = bsoncxx::builder::stream::document() << bsoncxx::builder::stream::finalize;
    std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

    bsoncxx::document::value projection = bsoncxx::builder::stream::document()
            << "_id" << 0
            << "group_id" << 0
            << bsoncxx::builder::stream::finalize;
    std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;

    mongocxx::options::find options = mongocxx::options::find().skip(start).limit(length).projection(projection.view());

    mongocxx::collection coll = db["faceset_group"];
    mongocxx::cursor cursor = coll.find(filter.view(), options);

    std::vector<bsoncxx::document::value> docs(cursor.begin(), cursor.end());
    for (bsoncxx::document::value doc : docs) {
        std::cout << "doc:" << bsoncxx::to_json(doc) << std::endl;
    }


    return docs;
}


int main() {

    mongocxx::instance instance{}; // This should be done only once.
    mongocxx::uri uri("mongodb://192.168.8.179:27017");

    client = mongocxx::client(uri);
    db = client["face"];

//    std::string group_id = bsoncxx::oid().to_string();
//    group_add(group_id,"group2");

    getlist(0, 100);


    return 0;
}


