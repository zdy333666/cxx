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
        std::vector<std::string> getlist(int start, int length) {

            bsoncxx::document::value filter =
                    bsoncxx::builder::stream::document() << bsoncxx::builder::stream::finalize;
            std::cout << "filter:" << bsoncxx::to_json(filter) << std::endl;

            bsoncxx::document::value projection = bsoncxx::builder::stream::document()
                    << "_id" << 0
                    << bsoncxx::builder::stream::finalize;
            std::cout << "projection:" << bsoncxx::to_json(projection) << std::endl;

            mongocxx::options::find options = mongocxx::options::find().skip(start).limit(length).projection(
                    projection.view());

            mongocxx::collection coll = db["faceset_group"];
            mongocxx::cursor cursor = coll.find(filter.view(), options);

            std::vector<bsoncxx::document::value> docs(cursor.begin(), cursor.end());

            std::vector<std::string> result;

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
         * @param start
         * @param length
         * @return
         */
        std::vector<std::string> getusers(std::string &group_id, int start, int length) {

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

            std::vector<bsoncxx::document::value> docs(cursor.begin(), cursor.end());

            std::vector<std::string> result;

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
            result = coll.delete_one(filter.view());
            std::cout << "faceset_user deleted count:" << result.value().deleted_count() << std::endl;

            coll = db["faceset_group"];
            result = coll.delete_one(filter.view());
            std::cout << "faceset_group deleted count:" << result.value().deleted_count() << std::endl;
        }

    }


    namespace user {

        /**
         *
         * @param image
         * @param group_id
         * @param user_id
         * @param user_info
         * @param action_type
         * @param descriptor
         * @return
         */
        std::string add(std::string &image_id, std::string &group_id, std::string &user_id, std::string &face_token,
                        std::string &user_info,
                        std::string action_type, char *descriptor) {


            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "image_id" << image_id
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << "user_info" << user_info
                    << "descriptor" << descriptor
                    << bsoncxx::builder::stream::finalize;

            std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;

            mongocxx::collection coll = db["faceset_user"];
            bsoncxx::stdx::optional<mongocxx::result::insert_one> result = coll.insert_one(doc.view());

            std::string inserted_id = result.value().inserted_id().get_oid().value.to_string();
            std::cout << "inserted_id:" << inserted_id << std::endl;

            return inserted_id;
        }

        /**
         *
         * @param image
         * @param group_id
         * @param user_id
         * @param user_info
         * @param action_type
         * @param descriptor
         */
        auto update(std::string &image, std::string &group_id, std::string &user_id, std::string &user_info,
                    std::string action_type, char *descriptor) {


            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "image" << image
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << "user_info" << user_info
                    << "descriptor" << descriptor
                    << bsoncxx::builder::stream::finalize;

            std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;

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
         * @return
         */
        auto get(std::string &group_id, std::string &user_id) {

        }

        /**
         *
         * @param user_id
         * @param src_group_id
         * @param dst_group_id
         * @return
         */
        auto copy(std::string &user_id, std::string &src_group_id, std::string &dst_group_id) {

        }

        /**
         *
         * @param group_id
         * @param user_id
         * @return
         */
        auto delete_one(std::string &group_id, std::string &user_id) {

            auto builder = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc = builder
                    << "group_id" << group_id
                    << "user_id" << user_id
                    << bsoncxx::builder::stream::finalize;

            std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;

            mongocxx::collection coll = db["faceset_user"];
            bsoncxx::stdx::optional<mongocxx::result::delete_result> result = coll.delete_one(doc.view());

            int deleted_count = result.value().deleted_count();
            std::cout << "deleted_count:" << deleted_count << std::endl;

            return deleted_count;
        }
    }


    namespace face {

        /**
         *
         * @param group_id
         * @param user_id
         * @return
         */
        auto getlist(std::string &group_id, std::string &user_id) {

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

            std::cout << "document:" << bsoncxx::to_json(doc) << std::endl;

            mongocxx::collection coll = db["faceset_face"];
            bsoncxx::stdx::optional<mongocxx::result::delete_result> result = coll.delete_one(doc.view());

            int deleted_count = result.value().deleted_count();
            std::cout << "deleted_count:" << deleted_count << std::endl;

            return deleted_count;

        }


    }

}