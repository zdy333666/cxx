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
    mongocxx::client client;
    mongocxx::database db;

    /**
     *
     */
    void init(const std::string& uri_str) {
        mongocxx::uri uri(uri_str);
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
//            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "user_id" << 1
                    << "user_info" << 1
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;

            mongocxx::options::find options = mongocxx::options::find().skip(start).limit(length).projection(
                    projection.view());

            mongocxx::collection coll = db["faceset_user"];
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
         * @return
         */
        void delete_one(std::string &group_id) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "group_id" << group_id
                    << bsoncxx::builder::stream::finalize;

//            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

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
//            std::cout << "doc:" << bsoncxx::to_json(doc) << std::endl;

            mongocxx::collection coll = db["faceset_user"];
            bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

            std::string inserted_id = result.value().inserted_id().get_oid().value.to_string();
            std::cout << "inserted id:" << inserted_id << std::endl;

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
//            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value update =  bsoncxx::builder::stream::document()
                    << "$set" << bsoncxx::builder::stream::open_document
                        << "user_info" << user_info
                        << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                    << bsoncxx::builder::stream::close_document
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "update:" << bsoncxx::to_json(update) << std::endl;

            mongocxx::options::update options;
            options.upsert(false);

            mongocxx::collection coll = db["faceset_user"];
            bsoncxx::stdx::optional<mongocxx::result::update> result = coll.update_one(filter.view(), update.view(), options);

            int upserted_count = result.value().result().upserted_count();
            std::cout << "upserted count:" << upserted_count << std::endl;

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
                        << "user_info" << bsoncxx::from_json(user_info)
                        << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                    << bsoncxx::builder::stream::close_document
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "update:" << bsoncxx::to_json(update) << std::endl;

            mongocxx::options::update options;
            options.upsert(true);

            mongocxx::collection coll = db["faceset_user"];
            bsoncxx::stdx::optional<mongocxx::result::update> result = coll.update_one(filter.view(), update.view(), options);

            int upserted_count = result.value().result().upserted_count();
            std::cout << "faceset user upserted count:" << upserted_count << std::endl;

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
//            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "group_id" << 1
                    << "user_info" << 1
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;

            mongocxx::options::find options;
            options.projection(projection.view());

            mongocxx::collection coll = db["faceset_user"];
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
         * @param location_json
         * @param landmark_jsons
         * @param descriptors
         * @return
         */
        std::string add(std::string &group_id, std::string &user_id, std::string &image_id, std::string &face_token, std::string &location_json
                , boost::container::stable_vector<std::string>* landmark_jsons, boost::container::stable_vector<double>* descriptors) {

            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << "image_id" << image_id
                    << "face_token" << face_token
                    << "location" << bsoncxx::from_json(location_json)
                    << "landmark" << bsoncxx::builder::stream::open_array
                    << [&](bsoncxx::builder::stream::array_context<> arr) {
                        for (std::string landmark_json : (*landmark_jsons)) {
                            arr <<  bsoncxx::from_json(landmark_json);
                        }
                    }
                    << bsoncxx::builder::stream::close_array
                    << "descriptor" << bsoncxx::builder::stream::open_array
                    << [&](bsoncxx::builder::stream::array_context<> arr) {
                        for (double descriptor : (*descriptors)) {
                            arr << descriptor;
                        }
                    }
                    << bsoncxx::builder::stream::close_array
                    << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                    << bsoncxx::builder::stream::finalize;

//            std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;

            mongocxx::collection coll = db["faceset_face"];
            bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

            std::string inserted_id = result.value().inserted_id().get_oid().value.to_string();
            std::cout << "faceset face inserted id:" << inserted_id << std::endl;

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

        std::string add(std::string& image_id, std::string& data_base64, std::string& user_id){

            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "image_id" << image_id
                    << "data_base64" << data_base64
                    << "user_id" << user_id
                    << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;


            mongocxx::collection coll = db["faceset_image"];
            bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

            std::string inserted_id = result.value().inserted_id().get_oid().value.to_string();
//            std::cout << "inserted_id:" << inserted_id << std::endl;

            return inserted_id;
        }

        /**
         *
         * @param image_id
         * @return
         */
        std::string get(std::string& image_id) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "image_id" << image_id
                    << bsoncxx::builder::stream::finalize;
            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;


            mongocxx::options::find options;
            options.projection(projection.view());

            mongocxx::collection coll = db["faceset_image"];
            bsoncxx::stdx::optional<bsoncxx::document::value> doc = coll.find_one(filter.view());

            std::string result = bsoncxx::to_json(doc.value());

            return result;
        }

        /**
         *
         * @param user_id
         * @return
         */
        boost::container::stable_vector<std::string> getlist(std::string& user_id) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;
            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;


            bsoncxx::document::value sort = bsoncxx::builder::stream::document()
                    << "ctime" << 1
                    << bsoncxx::builder::stream::finalize;
//            std::cout << "sort:" << bsoncxx::to_json(sort) << std::endl;


            mongocxx::options::find options;
            options.projection(projection.view());
            options.sort(sort.view());

            mongocxx::collection coll = db["faceset_image"];
            mongocxx::cursor cursor = coll.find(filter.view(), options);

            boost::container::stable_vector<std::string> result;

            for ( bsoncxx::document::view doc : cursor){
                std::string doc_json = bsoncxx::to_json(doc);
                result.push_back(doc_json);

//                std::cout << "doc:" << doc_json << std::endl;
            }

            return result;
        }

    }

}