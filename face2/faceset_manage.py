import datetime
import pymongo

client = pymongo.MongoClient("mongodb://localhost:27017/")
db = client["face2"]


# void init(const std::string& uri_str) {
#     mongocxx::uri uri(uri_str);
#     client = mongocxx::client(uri);
#     db = client["face"];
# }


class Group:

    def add(self, group_id, group_name):

        doc = dict()
        doc["group_id"] = group_id
        doc["group_name"] = group_name
        doc["ctime"] = datetime.datetime.utcnow()

        # print("document:", doc)

        _coll = db["faceset_group"]
        _result = _coll.insert_one(doc)

        inserted_id = _result.inserted_id
        print("inserted_id:", inserted_id)

        return inserted_id

    def getlist(self, start, length):

        _filter = dict()
        # print("filter:", _filter)

        projection = dict()
        projection["_id"] = 0
        projection["group_id"] = 1
        projection["group_name"] = 1

        sort = [("group_id", 1)]

        _coll = db["faceset_group"]
        _cursor = _coll.find(filter=_filter, projection=projection, sort=sort, skip=start, limit=length)

        _result = list()

        for doc in _cursor:
            _result.append(doc)

            # print("doc:", doc)

        return _result

    def getusers(self, group_id, start, length):

        _filter = dict()
        _filter["group_id"] = group_id

        print("filter:", _filter)

        projection = dict()
        projection["_id"] = 0
        projection["user_id"] = 1
        projection["user_info"] = 1

        _coll = db["faceset_user"]
        _cursor = _coll.find(filter=_filter, projection=projection, skip=start, limit=length)

        _result = list()

        for doc in _cursor:
            _result.append(doc)

            # print("doc:", doc)

        return _result

    def delete_one(self, group_id):

        _filter = dict()
        _filter["group_id"] = group_id

        print("filter:", _filter)

        _coll = db["faceset_face"]
        _result = _coll.delete_many(_filter)
        print("faceset_face deleted count:", _result.deleted_count)

        _coll = db["faceset_user"]
        _result = _coll.delete_many(_filter)
        print("faceset_user deleted count:", _result.deleted_count)

        _coll = db["faceset_group"]
        _result = _coll.delete_one(_filter)
        print("faceset_group deleted count:", _result.deleted_count)


#
class User:

    def insert(self, group_id, user_id, user_info):

        doc = dict()
        doc["group_id"] = group_id
        doc["user_id"] = user_id
        doc["user_info"] = user_info
        doc["ctime"] = datetime.datetime.utcnow()

        print('doc:', doc)

        _coll = db["faceset_user"]
        _result = _coll.insert_one(doc)

        inserted_id = _result.inserted_id
        print("faceset user inserted id:", inserted_id)

        return inserted_id

    def update(self, group_id, user_id, user_info):

        _filter = dict()
        _filter["group_id"] = group_id
        _filter["user_id"] = user_id

        print("filter:", _filter)

        _update = dict()
        _update["$set"] = {
            "user_info": user_info,
            "ctime": datetime.datetime.utcnow()
        }

        print("update:", _update)

        _coll = db["faceset_user"]
        _result = _coll.update_one(filter=_filter, update=_update, upsert=False)

        modified_count = _result.modified_count
        print("faceset user modified_count:", modified_count)

        return modified_count

    def upsert(self, group_id, user_id, user_info):

        _filter = dict()
        _filter["group_id"] = group_id
        _filter["user_id"] = user_id

        # print("filter:", _filter)

        _update = dict()
        _update["$set"] = {
            "user_info": user_info,
            "ctime": datetime.datetime.utcnow()
        }

        # print("update:", _update)

        _coll = db["faceset_user"]
        _result = _coll.update_one(filter=_filter, update=_update, upsert=True)

        modified_count = _result.modified_count
        print("faceset user modified count:", modified_count)

        return modified_count

    def get(self, group_id, user_id):

        _filter = dict()

        if group_id != "@ALL":
            _filter["group_id"] = group_id

        _filter["user_id"] = user_id

        print("filter:", _filter)

        projection = dict()
        projection["_id"] = 0
        projection["group_id"] = 1
        projection["user_info"] = 1

        print("projection:", projection)

        _coll = db["faceset_user"];
        _cursor = _coll.find(filter=_filter, projection=projection)

        result = list()

        for doc in _cursor:
            result.append(doc)
            # print("doc:", doc)

        return result

    def copy(self, user_id, src_group_id, dst_group_id):

        # copy user

        user_filter = dict()
        user_filter["group_id"] = src_group_id
        user_filter["user_id"] = user_id

        user_projection = dict()
        user_projection["_id"] = 0
        user_projection["user_info"] = 1

        user_coll = db["faceset_user"];
        user_cursor = user_coll.find(filter=user_filter, projection=user_projection)

        new_user_docs = list()

        for doc in user_cursor:
            new_user_doc = dict()
            new_user_doc["group_id"] = dst_group_id
            new_user_doc["user_id"] = user_id
            new_user_doc["user_info"] = doc["user_info"]
            new_user_doc["ctime"] = datetime.datetime.utcnow()

            new_user_docs.append(new_user_doc)

        if len(new_user_docs) > 0:
            user_coll.insert_many(new_user_docs)

        # copy image

        image_filter = dict()
        image_filter["group_id"] = src_group_id
        image_filter["user_id"] = user_id

        image_projection = dict()
        image_projection["_id"] = 0
        image_projection["image_id"] = 1
        image_projection["data_base64"] = 1

        image_coll = db["faceset_image"];
        image_cursor = image_coll.find(filter=image_filter, projection=image_projection)

        new_image_docs = list()

        for doc in image_cursor:
            new_image_doc = dict()
            new_image_doc["group_id"] = dst_group_id
            new_image_doc["user_id"] = user_id
            new_image_doc["image_id"] = doc["image_id"]
            new_image_doc["data_base64"] = doc["data_base64"]
            new_image_doc["ctime"] = datetime.datetime.utcnow()

            new_image_docs.append(new_user_doc)

        if len(new_image_docs) > 0:
            image_coll.insert_many(new_image_docs)

        # copy face

        face_filter = dict()
        face_filter["group_id"] = src_group_id
        face_filter["user_id"] = user_id

        face_projection = dict()
        face_projection["_id"] = 0
        face_projection["image_id"] = 1
        face_projection["face_token"] = 1
        face_projection["location"] = 1
        face_projection["landmark"] = 1
        face_projection["descriptor"] = 1

        face_coll = db["faceset_face"];
        face_cursor = face_coll.find(filter=face_filter, projection=face_projection)

        new_face_docs = list()

        for doc in face_cursor:
            new_face_doc = dict()
            new_face_doc["group_id"] = dst_group_id
            new_face_doc["user_id"] = doc["image_id"]
            new_face_doc["image_id"] = user_id
            new_face_doc["face_token"] = doc["face_token"]
            new_face_doc["location"] = doc["location"]
            new_face_doc["landmark"] = doc["landmark"]
            new_face_doc["descriptor"] = doc["descriptor"]
            new_face_doc["ctime"] = datetime.datetime.utcnow()

            new_face_docs.append(new_face_doc)

        if len(new_face_docs) > 0:
            face_coll.insert_many(new_face_docs)

    def delete_one(self, group_id, user_id):

        _filter = dict()
        _filter["group_id"] = group_id
        _filter["user_id"] = user_id

        _coll = db["faceset_face"]
        result = _coll.delete_many(filter=_filter)
        print("faceset_face deleted count:", result.deleted_count)

        _coll = db["faceset_user"]
        result = _coll.delete_one(filter=_filter)
        print("faceset_user deleted count:", result.deleted_count)


#
class Face:

    def add(self, group_id, user_id, image_id, face_token, location, landmark, descriptors):

        doc = dict()
        doc["group_id"] = group_id
        doc["user_id"] = user_id
        doc["image_id"] = image_id
        doc["face_token"] = face_token
        doc["location"] = location
        doc["landmark"] = landmark
        doc["descriptor"] = descriptors
        doc["ctime"] = datetime.datetime.utcnow()

        _coll = db["faceset_face"]
        result = _coll.insert_one(doc)

        inserted_id = result.inserted_id
        print("faceset face inserted id:", inserted_id)

        return inserted_id

    def getlist(self, group_id, user_id):

        _filter = dict()
        _filter["group_id"] = group_id
        _filter["user_id"] = user_id

        projection = dict()
        projection["_id"] = 0
        projection["face_token"] = 1
        projection["ctime"] = 1

        _coll = db["faceset_face"]
        _cursor = _coll.find(filter=_filter, projection=projection)

        result = list()

        for doc in _cursor:
            result.append(doc)

        return result

    def getlist(self, group_id):

        _filter = dict()
        _filter["group_id"] = group_id

        projection = dict()
        projection["_id"] = 0
        projection["user_id"] = 1
        projection["descriptor"] = 1

        sort = [("_id", 1)]

        coll = db["faceset_face"]
        cursor = coll.find(filter=_filter, projection=projection, sort=sort)

        result = list()

        for doc in cursor:
            result.append(doc)

        return result

    def delete_one(self, group_id, user_id, face_token):

        _filter = dict()
        _filter["group_id"] = group_id
        _filter["user_id"] = user_id
        _filter["face_token"] = face_token

        coll = db["faceset_face"]
        result = coll.delete_one(_filter)

        deleted_count = result.deleted_count
        print("faceset face deleted count:", deleted_count)

        return deleted_count


#
class Image:

    def add(self, image_id, data_base64, user_id, group_id):
        doc = dict()
        doc["image_id"] = image_id
        doc["data_base64"] = data_base64
        doc["group_id"] = group_id
        doc["user_id"] = user_id
        doc["ctime"] = datetime.datetime.utcnow()

        coll = db["faceset_image"]
        result = coll.insert_one(doc)

        inserted_id = result.inserted_id
        print("faceset image inserted id:", inserted_id)

        return inserted_id

    def get(self, image_id):
        _filter = dict()
        _filter["image_id"] = image_id

        projection = dict()
        projection["_id"] = 0

        coll = db["faceset_image"]
        doc = coll.find_one(filter=_filter, projection=projection)

        return doc

    def getlist(self, user_id, group_id):
        _filter = dict()
        _filter["group_id"] = group_id
        _filter["user_id"] = user_id

        projection = dict()
        projection["_id"] = 0

        sort = [("ctime", 1)]

        coll = db["faceset_image"]
        cursor = coll.find(filter=_filter, projection=projection, sort=sort)

        result = list()

        for doc in cursor:
            result.append(doc)

        return result
