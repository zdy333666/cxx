from flask import Flask
from flask import request
from flask import Response
from bson.objectid import ObjectId

import sys
import time
import base64
import json
import datetime
import threading

import face_recognition
import faceset_manage

app = Flask(__name__)


def nowTime():
    return int(round(time.time() * 1000))


class ComplexEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, datetime.datetime):
            return obj.strftime('%Y-%m-%d %H:%M:%S')
        elif isinstance(obj, datetime.date):
            return obj.strftime('%Y-%m-%d')
        else:
            return json.JSONEncoder.default(self, obj)


faceset_auth = faceset_manage.Auth()
faceset_group = faceset_manage.Group()
faceset_user = faceset_manage.User()
faceset_face = faceset_manage.Face()
faceset_image = faceset_manage.Image()

"""
build_group_face_index
"""


def build_group_face_index(rlock, group_id):
    rlock.acquire()

    face_recognition.delete_group_ann_index(group_id)

    face_infos = faceset_face.getlist_by_group(group_id)
    if len(face_infos) > 0:

        user_ids = list()
        feature_list = list()

        for face_info in face_infos:
            user_id = face_info["user_id"]
            features = face_info["descriptor"]

            user_ids.append(user_id)
            feature_list.append(features)

        face_recognition.build_group_index(group_id, user_ids, feature_list)

    rlock.release()


"""
build_all_face_index
"""


def build_all_face_index(rlock):
    rlock.acquire()

    # clear the previous ann index
    face_recognition.clear_ann_index()

    group_infos = faceset_group.getlist(0, sys.maxsize)

    for group_info in group_infos:
        group_id = group_info["group_id"]
        build_group_face_index(rlock, group_id)

    rlock.release()

    index_count = len(group_infos)

    print("build face index of group count:", index_count)

    return index_count


@app.route('/')
def hello_world():
    return 'Hello face!'


@app.route('/test', methods=['POST'])
def test():
    result = {}
    try:
        param = request.json
        print("param:", param)

        result = {
            'error_code': 0,
            'error_msg': 'SUCCESS',
            'describe': 'This is face recognition rest api'
        }
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
rebuild index
"""


@app.route('/face/v1/rebuild_index', methods=['POST'])
def rebuild_index():
    result = dict()
    try:
        # param = request.json
        # print("param:", param)

        index_count = build_all_face_index()

        result["error_code"] = 0
        result["error_msg"] = "SUCCESS"
        result["log_id"] = ObjectId().__str__()
        result["index_count"] = index_count
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
rebuild group index
"""


@app.route('/face/v1/rebuild_group_index', methods=['POST'])
def rebuild_group_index():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        group_id = param["group_id"]

        build_group_face_index(rlock, group_id)

        result["error_code"] = 0
        result["error_msg"] = "SUCCESS"
        result["log_id"] = ObjectId().__str__()

    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
人脸检测
"""


@app.route('/face/v1/detect', methods=['POST'])
def face_detect():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        image = param["image"]
        image_type = param["image_type"]
        # face_field = param["face_field"]
        max_face_num = param["max_face_num"]

        if image_type == "BASE64":
            start_time = nowTime()
            raw_image = base64.b64decode(image)
            end_time = nowTime()
            print("b64decode in ", end_time - start_time, " ms")

            print("image base64 size:", len(image))
            print("image raw_image size:", len(raw_image))

            # print("image raw_image type:", type(raw_image))

        log_id = ObjectId().__str__()

        loaded_img = face_recognition.local_load_image(raw_image, log_id)
        face_infos = face_recognition.detect(loaded_img, max_face_num)

        # print("face_infos", face_infos)

        # app.logger.debug('A value for debugging')

        face_list = list()
        for face_info in face_infos:
            face = dict()
            face["face_token"] = face_info['face_token']
            face["face_probability"] = face_info['face_probability']
            face["label"] = face_info['label']
            face["location"] = face_info['location']
            face["landmark"] = face_info['landmark']

            face_list.append(face)

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["log_id"] = log_id
        result["timestamp"] = nowTime()
        result["cached"] = 0
        result["result"] = {
            'face_num': len(face_list),
            'face_list': face_list
        }
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
人脸搜索
"""


@app.route('/face/v1/search', methods=['POST'])
def face_search():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        image = param["image"]
        image_type = param["image_type"]
        group_id_list = param["group_id_list"]
        max_user_num = param["max_user_num"]

        if image_type == "BASE64":
            start_time = nowTime()
            raw_image = base64.b64decode(image)
            end_time = nowTime()
            print("b64decode in ", end_time - start_time, " ms")

            print("image base64 size:", len(image))
            print("image raw_image size:", len(raw_image))

            # print("image raw_image type:", type(raw_image))

        log_id = ObjectId().__str__()

        loaded_img = face_recognition.local_load_image(raw_image, log_id)
        face_infos = face_recognition.detect(loaded_img, 1)

        # print("face_infos", face_infos)

        if len(face_infos) > 0:
            face_info = face_infos[0]

            face_token = face_info['face_token']
            descriptors = face_info['descriptors']

            group_ids = group_id_list.split(',')

            print("group_ids:", group_ids)

            user_infos = face_recognition.user_search(descriptors, max_user_num, group_ids)

            user_list = list()

            for user_info in user_infos:

                user_doc_list = faceset_user.get(user_info["group_id"], user_info["user_id"])

                if len(user_doc_list) > 0:
                    user_doc = user_doc_list[0]
                    user_info_json = user_doc["user_info"]

                    user_json = dict()
                    user_json["group_id"] = user_info['group_id']
                    user_json["user_id"] = user_info['user_id']
                    user_json["user_info"] = user_info_json
                    user_json["score"] = user_info['score']

                    user_list.append(user_json)

            result["error_code"] = 0
            result["error_msg"] = "SUCCESS"
            result["log_id"] = ObjectId().__str__()
            result["face_token"] = face_token
            result["user_list"] = user_list
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
 创建用户组
"""


@app.route('/face/v1/faceset/group/add', methods=['POST'])
def faceset_group_add():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        group_id = param["group_id"]
        group_name = param["group_name"]

        faceset_group.add(group_id, group_name)

        result["error_code"] = 0
        result["error_msg"] = "SUCCESS"
        result["log_id"] = ObjectId().__str__()
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
组列表查询
"""


@app.route('/face/v1/faceset/group/getlist', methods=['POST'])
def faceset_group_getlist():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        start = param["start"]
        length = param["length"]

        groups = faceset_group.getlist(start, length)

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["log_id"] = ObjectId().__str__()
        result["group_id_list"] = groups
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
取用户列表
"""


@app.route('/face/v1/faceset/group/getusers', methods=['POST'])
def faceset_group_getusers():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        group_id = param["group_id"]
        start = param["start"]
        length = param["length"]

        users = faceset_group.getusers(group_id, start, length)

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["log_id"] = ObjectId().__str__()
        result["user_id_list"] = users
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
删除用户组
"""


@app.route('/face/v1/faceset/group/delete', methods=['POST'])
def faceset_group_delete():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        group_id = param["group_id"]

        faceset_group.delete_one(group_id)

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["log_id"] = ObjectId().__str__()

        # delete group ann index
        face_recognition.delete_group_ann_index(group_id)

    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
人脸注册
"""


@app.route('/face/v1/faceset/user/add', methods=['POST'])
def faceset_user_add():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        image = param["image"]
        image_type = param["image_type"]
        group_id = param["group_id"]
        user_id = param["user_id"]
        user_info = param["user_info"]
        # quality_control = param["quality_control"]
        # liveness_control = param["liveness_control"]
        action_type = param["action_type"]

        if image_type == "BASE64":
            start_time = nowTime()
            raw_image = base64.b64decode(image)
            end_time = nowTime()
            print("b64decode in ", end_time - start_time, " ms")

            print("image base64 size:", len(image))
            print("image raw_image size:", len(raw_image))

        log_id = ObjectId().__str__()

        loaded_img = face_recognition.local_load_image(raw_image, log_id)
        face_infos = face_recognition.detect(loaded_img, 1)

        if len(face_infos) > 0:

            faceset_image.add(log_id, image, user_id, group_id)

            face_info = face_infos[0]

            face_token = face_info["face_token"]
            location = face_info["location"]
            landmark = face_info["landmark"]
            descriptors = face_info["descriptors"]

            if action_type == "APPEND":
                pass
            elif action_type == "REPLACE":
                faceset_user.delete_one(group_id, user_id)

            user_info_json = json.loads(user_info)

            faceset_user.upsert(group_id, user_id, user_info_json)
            faceset_face.add(group_id, user_id, log_id, face_token, location, landmark, descriptors)

            result["error_code"] = 0
            result["error_msg"] = 'SUCCESS'
            result["log_id"] = ObjectId().__str__()
            result["result"] = {
                "face_token": face_token,
                "location": location
            }

            # rebuild group index
            build_group_face_index(rlock, group_id)

    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
人脸更新
"""


@app.route('/face/v1/faceset/user/update', methods=['POST'])
def faceset_user_update():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        image = param["image"]
        image_type = param["image_type"]
        group_id = param["group_id"]
        user_id = param["user_id"]
        user_info = param["user_info"]
        # quality_control = param["quality_control"]
        # liveness_control = param["liveness_control"]
        action_type = param["action_type"]

        if image_type == "BASE64":
            start_time = nowTime()
            raw_image = base64.b64decode(image)
            end_time = nowTime()
            print("b64decode in ", end_time - start_time, " ms")

            print("image base64 size:", len(image))
            print("image raw_image size:", len(raw_image))

        log_id = ObjectId().__str__()

        loaded_img = face_recognition.local_load_image(raw_image, log_id)
        face_infos = face_recognition.detect(loaded_img, 1)

        if len(face_infos) > 0:

            faceset_image.add(log_id, image, user_id, group_id)

            face_info = face_infos[0]

            face_token = face_info["face_token"]
            location = face_info["location"]
            landmark = face_info["landmark"]
            descriptors = face_info["descriptors"]

            if action_type == "UPDATE":
                rows = faceset_user.get(group_id, user_id)
                if len(rows) > 0:
                    faceset_user.delete_one(group_id, user_id)
                    faceset_user.upsert(group_id, user_id, user_info)
                    faceset_face.add(group_id, user_id, log_id, face_token, location, landmark, descriptors)

            elif action_type == "REPLACE":
                faceset_user.delete_one(group_id, user_id)
                faceset_user.upsert(group_id, user_id, user_info)
                faceset_face.add(group_id, user_id, log_id, face_token, location, landmark, descriptors)

            result["error_code"] = 0
            result["error_msg"] = 'SUCCESS'
            result["log_id"] = ObjectId().__str__()
            result["result"] = {
                "face_token": face_token,
                "location": location
            }

            # rebuild group index
            build_group_face_index(rlock, group_id)

    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()
        result["log_id"] = ObjectId().__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
用户信息查询
"""


@app.route('/face/v1/faceset/user/get', methods=['POST'])
def faceset_user_get():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        group_id = param["group_id"]
        user_id = param["user_id"]

        rows = faceset_user.get(group_id, user_id)

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["log_id"] = ObjectId().__str__()
        result["user_list"] = rows
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["log_id"] = ObjectId().__str__()
        result["error_msg"] = (sys.exc_info()[1]).__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
复制用户
"""


@app.route('/face/v1/faceset/user/copy', methods=['POST'])
def faceset_user_copy():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        user_id = param["user_id"]
        src_group_id = param["src_group_id"]
        dst_group_id = param["dst_group_id"]

        faceset_user.copy(user_id, src_group_id, dst_group_id)

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["log_id"] = ObjectId().__str__()

        # rebuild group index
        build_group_face_index(rlock, dst_group_id)

    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["log_id"] = ObjectId().__str__()
        result["error_msg"] = (sys.exc_info()[1]).__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
复制用户
"""


@app.route('/face/v1/faceset/user/delete', methods=['POST'])
def faceset_user_delete():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        group_id = param["group_id"]
        user_id = param["user_id"]

        faceset_user.delete_one(group_id, user_id)

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["log_id"] = ObjectId().__str__()

        # rebuild group index
        build_group_face_index(rlock, group_id)

    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["log_id"] = ObjectId().__str__()
        result["error_msg"] = (sys.exc_info()[1]).__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
获取用户人脸列表
"""


@app.route('/face/v1/faceset/face/getlist', methods=['POST'])
def faceset_face_getlist():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        group_id = param["group_id"]
        user_id = param["user_id"]

        rows = faceset_face.getlist(group_id, user_id)

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["log_id"] = ObjectId().__str__()
        result["face_list"] = rows
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["log_id"] = ObjectId().__str__()
        result["error_msg"] = (sys.exc_info()[1]).__str__()

    resp = Response(json.dumps(result, cls=ComplexEncoder))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
人脸删除
"""


@app.route('/face/v1/faceset/face/delete', methods=['POST'])
def faceset_face_delete():
    result = dict()
    try:
        param = request.json
        # print("param:", param)

        group_id = param["group_id"]
        user_id = param["user_id"]
        face_token = param["face_token"]

        faceset_face.delete_one(group_id, user_id, face_token)

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["log_id"] = ObjectId().__str__()

        # rebuild group index
        build_group_face_index(rlock, group_id)

    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["log_id"] = ObjectId().__str__()
        result["error_msg"] = (sys.exc_info()[1]).__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
get access_token
"""


@app.route('/oauth/2.0/token', methods=['POST'])
def get_token():
    result = dict()
    try:
        token = faceset_auth.get_token()

        result["error_code"] = 0
        result["error_msg"] = 'SUCCESS'
        result["access_token"] = token
        result["expires_in"] = 3600 * 24 * 30
    except Exception:
        print("Unexpected error:", sys.exc_info())

        result.clear()
        result["error_code"] = 1
        result["error_msg"] = (sys.exc_info()[1]).__str__()

    resp = Response(json.dumps(result))
    resp.headers['Content-Type'] = 'application/json; charset=utf-8'

    return resp


"""
before_request
"""


@app.before_request
def before_request():
    print("before request ...")

    # if request.path == '/oauth/2.0/token':
    #     return
    #
    # access_token = request.args.get('access_token', None)
    # if (access_token is None) or (not faceset_auth.check_token(access_token)):
    #     result = dict()
    #     result["error_code"] = 1
    #     result["error_msg"] = "invalid access_token"
    #
    #     resp = Response(json.dumps(result))
    #     resp.headers['Content-Type'] = 'application/json; charset=utf-8'
    #
    #     return resp


"""
after_request
"""


@app.after_request
def after_request(response):
    print("after request ...")
    print()

    return response


if __name__ == '__main__':
    rlock = threading.RLock()

    # face search index ready
    build_all_face_index(rlock)

    app.config["SERVER_NAME"] = "192.168.8.120:8080"
    app.run()
