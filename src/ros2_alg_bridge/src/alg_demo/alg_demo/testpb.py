

import test_pb2
from std_msgs.msg import String, ByteMultiArray

t = test_pb2.Test()

t.name = "ll"
s = t.SerializeToString()

s1 = String()
s1.data = s.decode()

r = s1.data.encode()

t.ParseFromString(r)

b = ByteMultiArray()
# b.data = s
print(s1)

print(t)
print(s)