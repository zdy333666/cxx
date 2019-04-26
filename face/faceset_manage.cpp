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

            // copy user

            bsoncxx::document::value user_filter =  bsoncxx::builder::stream::document()
                    << "group_id" << src_group_id
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;

            bsoncxx::document::value user_projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "user_info" << 1
                    << bsoncxx::builder::stream::finalize;

            mongocxx::options::find user_options;
            user_options.projection(user_projection.view());

            mongocxx::collection user_coll = db["faceset_user"];
            mongocxx::cursor user_cursor = user_coll.find(user_filter.view(), user_options);

            boost::container::stable_vector<bsoncxx::document::value> new_user_docs;

            for ( bsoncxx::document::view doc : user_cursor){

                bsoncxx::document::value new_user_doc =  bsoncxx::builder::stream::document()
                        << "group_id" << dst_group_id
                        << "user_id" << user_id
                        << "user_info" << doc["user_info"].get_value()
                        << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                        << bsoncxx::builder::stream::finalize;

                new_user_docs.push_back(new_user_doc);
            }

            if(! new_user_docs.empty()){
                user_coll.insert_many(new_user_docs);
            }


            // copy image

            bsoncxx::document::value image_filter =  bsoncxx::builder::stream::document()
                    << "group_id" << src_group_id
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;

            bsoncxx::document::value image_projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "image_id" << 1
                    << "data_base64" << 1
                    << bsoncxx::builder::stream::finalize;

            mongocxx::options::find image_options;
            image_options.projection(image_projection.view());

            mongocxx::collection image_coll = db["faceset_image"];
            mongocxx::cursor image_cursor = image_coll.find(image_filter.view(), image_options);


            boost::container::stable_vector<bsoncxx::document::value> new_image_docs;

            for ( bsoncxx::document::view doc : image_cursor){

                bsoncxx::document::value new_image_doc =  bsoncxx::builder::stream::document()
                        << "group_id" << dst_group_id
                        << "user_id" << user_id
                        << "image_id" << doc["image_id"].get_value()
                        << "data_base64" << doc["data_base64"].get_value()
                        << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                        << bsoncxx::builder::stream::finalize;

                new_image_docs.push_back(new_image_doc);
            }

            if(! new_image_docs.empty()){
                image_coll.insert_many(new_image_docs);
            }


            // copy face

            bsoncxx::document::value face_filter =  bsoncxx::builder::stream::document()
                    << "group_id" << src_group_id
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;

            bsoncxx::document::value face_projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << "image_id" << 1
                    << "face_token" << 1
                    << "location" << 1
                    << "landmark" << 1
                    << "descriptor" << 1
                    << bsoncxx::builder::stream::finalize;

            mongocxx::options::find face_options;
            face_options.projection(face_projection.view());

            mongocxx::collection face_coll = db["faceset_face"];
            mongocxx::cursor face_cursor = face_coll.find(face_filter.view(), face_options);


            boost::container::stable_vector<bsoncxx::document::value> new_face_docs;

            for ( bsoncxx::document::view doc : face_cursor){

                bsoncxx::document::value new_face_doc =  bsoncxx::builder::stream::document()
                        << "group_id" << dst_group_id
                        << "user_id" << user_id
                        << "image_id" << doc["image_id"].get_value()
                        << "face_token" << doc["face_token"].get_value()
                        << "location" << doc["location"].get_value()
                        << "landmark" << doc["landmark"].get_value()
                        << "descriptor" << doc["descriptor"].get_value()
                        << "ctime" <<  bsoncxx::types::b_date(std::chrono::system_clock::now())
                        << bsoncxx::builder::stream::finalize;

                new_face_docs.push_back(new_face_doc);
            }

            if(! new_face_docs.empty()){
                face_coll.insert_many(new_face_docs);
            }
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

        std::string add(std::string& image_id, std::string& data_base64, std::string& user_id, std::string& group_id){

            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "image_id" << image_id
                    << "data_base64" << data_base64
                    << "group_id" << group_id
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
        boost::container::stable_vector<std::string> getlist(std::string& user_id, std::string& group_id) {

            bsoncxx::document::value filter = bsoncxx::builder::stream::document()
                    << "group_id" << group_id
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