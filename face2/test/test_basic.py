# list = [1,2,4,5]
# # for n in list:
# #     print(n)
#
# aa = [x for n in list]
# print("aa:", aa)

import datetime

now = datetime.datetime.utcnow()

print("now:", now)

expired = datetime.datetime.fromtimestamp(now.timestamp()+(3600*24*30))

print("expired:", expired)
