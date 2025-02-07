import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message
from robot_custom_msgs.msg import TofData  # 실제 메시지 타입으로 변경

def extract_tof_data(bag_path, topic_name, num_samples=30):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')  # 사용된 포맷 확인 필요
    converter_options = rosbag2_py.ConverterOptions()
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    selected_data = []
    
    while reader.has_next() and len(selected_data) < num_samples:
        topic, data, _ = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, TofData)
            selected_data.append(msg.bot_right[12:16])
    
    return selected_data

if __name__ == "__main__":
    rclpy.init()
    bag_file = "/home/hyjoe/custom_ws/bag/0207/tof_1"  # 실제 경로 확인 필요
    topic = "/tof_data"
    data = extract_tof_data(bag_file, topic)
    for i, values in enumerate(data):
        print(f"Sample {i+1}: [{', '.join(map(str, values))}]")
    rclpy.shutdown()
