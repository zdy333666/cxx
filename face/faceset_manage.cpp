//
// Created by zdy on 19-4-9.
//

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>

namespace faceset {

    mongocxx::instance instance{}; // This should be done only once.
    mongocxx::uri uri("mongodb://192.168.8.179:27017");
    mongocxx::client client;
    mongocxx::database db;

    /**
     *
     */
    void init() {

        client = mongocxx::client(uri);
        db = client["face"];
    }


    namespace group {

        /**
         *
         * @param group_id
         * @return
         */
        std::string add(std::string &group_id, std::string &group_name) {

            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "group_id" << group_id
                    << "group_name" << group_name
                    << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
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
        boost::container::stable_vector<std::string> getlist(int start, int length) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document() << bsoncxx::builder::stream::finalize;
//            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "group_id" << 1
                    << "group_name" << 1
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;

            mongocxx::options::find options = mongocxx::options::find().skip(start).limit(length).projection( projection.view());

            mongocxx::collection coll = db["faceset_group"];
            mongocxx::cursor cursor = coll.find(filter.view(), options);

            boost::container::stable_vector<std::string> result;

            for ( bsoncxx::document::view doc : cursor){
                std::string doc_json = bsoncxx::to_json(doc);
                result.push_back(doc_json);

//                std::cout << "doc:" << doc_json << std::endl;
            }

            return result;
        }

        /**
         *
         * @param group_id
         * @param start
         * @param length
         * @return
         */
        boost::container::stable_vector<std::string> getusers(std::string &group_id, int start, int length) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "group_id" << group_id
                    << bsoncxx::builder::stream::finalize;
            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "user_id" << 1
                    << "user_info" << 1
                    << bsoncxx::builder::stream::finalize;
            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;

            mongocxx::options::find options = mongocxx::options::find().skip(start).limit(length).projection(
                    projection.view());

            mongocxx::collection coll = db["faceset_user"];
            mongocxx::cursor cursor = coll.find(filter.view(), options);

            boost::container::stable_vector<bsoncxx::document::value> docs(cursor.begin(), cursor.end());

            boost::container::stable_vector<std::string> result;

            for (bsoncxx::document::value doc : docs) {
                std::string doc_json = bsoncxx::to_json(doc);
                result.push_back(doc_json);

                std::cout << "doc:" << doc_json << std::endl;
            }

            return result;
        }

        /**
         *
         * @param group_id
         * @return
         */
        void delete_one(std::string &group_id) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "group_id" << group_id
                    << bsoncxx::builder::stream::finalize;

            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            mongocxx::collection coll = db["faceset_face"];
            bsoncxx::stdx::optional<mongocxx::result::delete_result> result = coll.delete_many(filter.view());
            std::cout << "faceset_face deleted count:" << result.value().deleted_count() << std::endl;

            coll = db["faceset_user"];
            result = coll.delete_many(filter.view());
            std::cout << "faceset_user deleted count:" << result.value().deleted_count() << std::endl;

            coll = db["faceset_group"];
            result = coll.delete_one(filter.view());
            std::cout << "faceset_group deleted count:" << result.value().deleted_count() << std::endl;
        }

    }


    namespace user {

        /**
         *
         * @param group_id
         * @param user_id
         * @param user_info
         * @return
         */
        std::string insert(std::string &group_id, std::string &user_id, std::string &user_info) {

            bsoncxx::document::value doc =  bsoncxx::builder::stream::document()
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << "user_info" << user_info
                    << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                    << bsoncxx::builder::stream::finalize;
            std::cout << "doc:" << bsoncxx::to_json(doc) << std::endl;

            mongocxx::collection coll = db["faceset_user"];
            bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

            std::string inserted_id = result.value().inserted_id().get_oid().value.to_string();
            std::cout << "inserted_id:" << inserted_id << std::endl;

            return inserted_id;
        }

        /**
         *
         * @param group_id
         * @param user_id
         * @param user_info
         * @return
         */
        int update(std::string &group_id, std::string &user_id, std::string &user_info) {

            bsoncxx::document::value filter =  bsoncxx::builder::stream::document()
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;
            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value update =  bsoncxx::builder::stream::document()
                    << "$set" << bsoncxx::builder::stream::open_document
                    << "user_info" << user_info
                    << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                    << bsoncxx::builder::stream::close_document
                    << bsoncxx::builder::stream::finalize;
            std::cout << "update:" << bsoncxx::to_json(update) << std::endl;

            mongocxx::options::update options;
            options.upsert(false);

            mongocxx::collection coll = db["faceset_user"];
            bsoncxx::stdx::optional<mongocxx::result::update> result = coll.update_one(filter.view(), update.view(), options);

            int upserted_count = result.value().result().upserted_count();
            std::cout << "upserted_count:" << upserted_count << std::endl;

            return upserted_count;
        }

        /**
         *
         * @param group_id
         * @param user_id
         * @param user_info
         * @return
         */
        int upsert(std::string &group_id, std::string &user_id, std::string &user_info) {

            bsoncxx::document::value filter =  bsoncxx::builder::stream::document()
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value update =  bsoncxx::builder::stream::document()
                    << "$set" << bsoncxx::builder::stream::open_document
//                    << "group_id" << group_id
//                    << "user_id" << user_id
                    << "user_info" << bsoncxx::from_json(user_info) //user_info
                    << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                    << bsoncxx::builder::stream::close_document
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "update:" << bsoncxx::to_json(update) << std::endl;

            mongocxx::options::update options;
            options.upsert(true);

            mongocxx::collection coll = db["faceset_user"];
            bsoncxx::stdx::optional<mongocxx::result::update> result = coll.update_one(filter.view(), update.view(), options);

            int upserted_count = result.value().modified_count();
            std::cout << "faceset user upserted_count:" << upserted_count << std::endl;

            return upserted_count;
        }

        /**
         *
         * @param group_id
         * @param user_id
         * @return
         */
        boost::container::stable_vector<std::string> get(std::string &group_id, std::string &user_id) {

            auto filter_builder = bsoncxx::builder::stream::document{};

            if(group_id != "@ALL"){
                filter_builder  << "group_id" << group_id;
            }

            bsoncxx::document::value filter = filter_builder
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;
            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "group_id" << 1
                    << "user_info" << 1
                    << bsoncxx::builder::stream::finalize;
            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;

            mongocxx::options::find options;
            options.projection(projection.view());

            mongocxx::collection coll = db["faceset_user"];
            mongocxx::cursor cursor = coll.find(filter.view(), options);

            boost::container::stable_vector<bsoncxx::document::value> docs(cursor.begin(), cursor.end());

            boost::container::stable_vector<std::string> result;

            for (bsoncxx::document::value doc : docs) {
                std::string doc_json = bsoncxx::to_json(doc);
                std::cout << "doc:" << doc_json << std::endl;

                result.push_back(doc_json);
            }

            return result;
        }

        /**
         *
         * @param user_id
         * @param src_group_id
         * @param dst_group_id
         * @return
         */
        void copy(std::string &user_id, std::string &src_group_id, std::string &dst_group_id) {

        }

        /**
         *
         * @param group_id
         * @param user_id
         * @return
         */
        void delete_one(std::string &group_id, std::string &user_id) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            mongocxx::collection coll = db["faceset_face"];
            bsoncxx::stdx::optional<mongocxx::result::delete_result> result = coll.delete_many(filter.view());
            std::cout << "faceset_face deleted count:" << result.value().deleted_count() << std::endl;

            coll = db["faceset_user"];
            result = coll.delete_one(filter.view());
            std::cout << "faceset_user deleted count:" << result.value().deleted_count() << std::endl;
        }

    }


    namespace face {

        /**
         *
         * @param group_id
         * @param user_id
         * @param image_id
         * @param face_token
         * @param descriptor
         * @return
         */
        std::string add(std::string &group_id, std::string &user_id, std::string &image_id, std::string &face_token, double *descriptors) {

            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << "image_id" << image_id
                    << "face_token" << face_token
                    << "descriptor" << bsoncxx::builder::stream::open_array
                    << [&](bsoncxx::builder::stream::array_context<> arr) {
                         int length = sizeof(descriptors)/ sizeof(double);
                        for ( int i = 0; i < length; i++) {
                            arr << descriptors[i];
                        }
                    }
                    << bsoncxx::builder::stream::close_array
                    << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                    << bsoncxx::builder::stream::finalize;

//            std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;

            mongocxx::collection coll = db["faceset_face"];
            bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

            std::string inserted_id = result.value().inserted_id().get_oid().value.to_string();
            std::cout << "faceset face inserted_id:" << inserted_id << std::endl;

            return inserted_id;
        }

        /**
         *
         * @param group_id
         * @param user_id
         * @return
         */
        boost::container::stable_vector<std::string> getlist(std::string &group_id, std::string &user_id) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "face_token" << 1
                    << "ctime" << 1
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;


            mongocxx::options::find options;
            options.projection(projection.view());

            mongocxx::collection coll = db["faceset_face"];
            mongocxx::cursor cursor = coll.find(filter.view(), options);

//            boost::container::stable_vector<bsoncxx::document::value> docs(cursor.begin(), cursor.end());

            boost::container::stable_vector<std::string> result;

//            for ( bsoncxx::document::view doc : cursor){
////                bsoncxx::document::view doc = *it;
//                std::string doc_json = bsoncxx::to_json(doc);
////                std::cout << "doc:" << doc_json << std::endl;
//
//                result.push_back(doc_json);
//            }
//

            for ( mongocxx::cursor::iterator it = cursor.begin(); it != cursor.end(); it++){
                    bsoncxx::document::view doc = *it;
                std::string doc_json = bsoncxx::to_json(doc);
//                std::cout << "doc:" << doc_json << std::endl;

                result.push_back(doc_json);
            }

            return result;
        }


        /**
         *
         * @param group_id
         * @return
         */
        boost::container::stable_vector<std::string> getlist(std::string &group_id) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "group_id" << group_id
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 1
                    << "user_id" << 1
                    << "descriptor" << 1
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;

            bsoncxx::document::value sort = bsoncxx::builder::stream::document()
                    << "_id" << 1
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "sort:" << bsoncxx::to_json(sort) << std::endl;


            mongocxx::options::find options;
            options.projection(projection.view());
            options.sort(sort.view());

            mongocxx::collection coll = db["faceset_face"];
            mongocxx::cursor cursor = coll.find(filter.view(), options);

            boost::container::stable_vector<bsoncxx::document::value> docs(cursor.begin(), cursor.end());

            boost::container::stable_vector<std::string> result;

            for (bsoncxx::document::value doc : docs) {
                std::string doc_json = bsoncxx::to_json(doc);
//                std::cout << "doc:" << doc_json << std::endl;

                result.push_back(doc_json);
            }

            return result;
        }

        /**
         *
         * @param group_id
         * @param user_id
         * @param face_token
         * @return
         */
        int delete_one(std::string &group_id, std::string &user_id, std::string &face_token) {

            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << "face_token" << face_token
                    << bsoncxx::builder::stream::finalize;

//            std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;


            mongocxx::collection coll = db["faceset_face"];
            bsoncxx::stdx::optional<mongocxx::result::delete_result> result = coll.delete_one(doc.view());

            int deleted_count = result.value().deleted_count();
            std::cout << "faceset face deleted count:" << deleted_count << std::endl;

            return deleted_count;
        }

    }


    namespace image {

        std::string add(std::string& image_id, std::string& data_base64){

            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "image_id" << image_id
                    << "data_base64" << data_base64
                    << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;


            mongocxx::collection coll = db["faceset_image"];
            bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

            std::string inserted_id = result.value().inserted_id().get_oid().value.to_string();
//            std::cout << "inserted_id:" << inserted_id << std::endl;

            return inserted_id;
        }


        std::string get(std::string& image_id){

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "image_id" << image_id
                    << bsoncxx::builder::stream::finalize;
            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "data_base64" << 1
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;


            mongocxx::options::find options;
            options.projection(projection.view());

            mongocxx::collection coll = db["faceset_image"];
            bsoncxx::stdx::optional<bsoncxx::document::value> doc = coll.find_one(filter.view());

            std::string result = bsoncxx::to_json(doc.value());

           return result;
        }

    }

}