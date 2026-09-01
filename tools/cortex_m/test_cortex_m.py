import gc, sys
print("=== SnakeROS on bare-metal ARM Cortex-M (MPS2-AN385) ===")
print("impl:", sys.implementation.name, sys.implementation.version)
gc.collect(); print("heap total:", gc.mem_free()+gc.mem_alloc())
from snakeros.cdr import CDRWriter, CDRReader
w = CDRWriter(); w.u8(1); w.f64(2.5)
print("CDR u8+f64 :", w.bytes().hex(), "len", len(w.bytes()))
w2 = CDRWriter(); w2.string("cortex-m")
print("CDR string :", w2.bytes().hex())
from snakeros.msg.geometry_msgs import Twist, Vector3
from snakeros.msg.sensor_msgs import Imu
from snakeros.msg.std_msgs import String
print("Vector3 fast fmt:", Vector3._fast())
t = Twist(); t.linear.x = 0.5; t.angular.z = -1.25
b = t.serialize()
print("Twist bytes:", len(b), b.hex())
back = Twist.deserialize(b)
print("Twist roundtrip:", back.linear.x == 0.5 and back.angular.z == -1.25)
imu = Imu(); imu.header.frame_id = "imu_link"; imu.orientation_covariance = [0.5]*9
ib = imu.serialize()
print("Imu bytes:", len(ib), "roundtrip:", Imu.deserialize(ib).orientation_covariance == [0.5]*9)
s = String(data="hello from Cortex-M")
print("String roundtrip:", String.deserialize(s.serialize()).data)
from snakeros.xrce.entities import object_id, parse_object_id, mangle_topic, dds_type_name, topic_xml
print("objid(300,2):", object_id(300,2).hex(), "->", parse_object_id(object_id(300,2)))
print("mangle:", mangle_topic("chatter"), "|", dds_type_name("std_msgs","msg","String"))
print("xml:", topic_xml(mangle_topic("chatter"), dds_type_name("std_msgs","msg","String"))[:60], "...")
gc.collect(); print("free heap after:", gc.mem_free())
print("=== CORTEX-M OK ===")
