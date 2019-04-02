//
// Created by zdy on 19-4-2.
//


#include <iostream>

#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>

int main() {


    auto builder = bsoncxx::builder::stream::document{};
    bsoncxx::document::value doc_value = builder
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


    std::cout << "document:" << bsoncxx::to_json(doc_value) << std::endl;


    return 0;
}