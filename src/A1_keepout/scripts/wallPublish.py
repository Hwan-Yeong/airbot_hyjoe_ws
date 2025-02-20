import rclpy
from rclpy.node import Node
from robot_custom_msgs.msg import BlockAreaList, BlockArea, Position
import yaml

class BlocksPublisher(Node):

    def __init__(self):
        super().__init__('blocks_publisher')
        self.publisher_ = self.create_publisher(BlockAreaList, '/block_walls', 10)
        # YAML 파일에서 데이터 로드
        self.blocks_data = self.load_yaml_data()
        self.publish_blocks();

    def load_yaml_data(self):
        # YAML 데이터를 불러옵니다.
        yaml_data = """
        - id: 1
          robot_path:
            points:
              - x: 0
                y: 0
              - x: 5
                y: 0
          image_path:
            points:
              - x: 0
                y: 0
              - x: 5
                y: 0
        - id: 2
          robot_path:
            points:
              - x: 5
                y: 5
              - x: 10
                y: 5
          image_path:
            points:
              - x: 0
                y: 0
              - x: 5
                y: 0
        """
        # YAML 문자열을 파싱합니다
        return yaml.safe_load(yaml_data)

    def publish_blocks(self):
        block_msg_list = BlockAreaList()
        for block_data in self.blocks_data:
            block_msg = BlockArea()
            block_msg.id = str(block_data['id'])
            
            # robot_path 변환
            for point in block_data['robot_path']['points']:
                robot_point = Position()
                robot_point.x = float(point['x'])
                robot_point.y = float(point['y'])
                block_msg.robot_path.append(robot_point)
            
            # image_path 변환
            for point in block_data['image_path']['points']:
                image_point = Position()
                image_point.x = float(point['x'])
                image_point.y = float(point['y'])
                block_msg.image_path.append(image_point)
            block_msg_list.block_area_list.append(block_msg)

        # 퍼블리시
        self.publisher_.publish(block_msg_list)
        self.get_logger().info(f"Published Block wall")


def main(args=None):
    rclpy.init(args=args)
    blocks_publisher = BlocksPublisher()
    rclpy.spin(blocks_publisher)
    blocks_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
