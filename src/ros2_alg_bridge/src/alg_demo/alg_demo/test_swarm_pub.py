import time
import rclpy
from rclpy.node import Node
from fq.msg import ActorDroneSwarm, ActorDroneSwarms, BaseReconnaissanceData, BaseTargetInfo, BaseInterferenceData, BaseReconnaissanceData
from api import FqTopic

class ProducerNode(Node):
    def __init__(self):
        super().__init__('producer_node')
        self.publisher_ = self.create_publisher(ActorDroneSwarms, FqTopic.drone_swarm_param.value, 10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        drone_swarms = ActorDroneSwarms()
        num = 0
        for i in range(10):
            msg = ActorDroneSwarm()
            msg.base_data.id = i
            msg.base_data.health_point = 20
            msg.base_data.type_id = "airplane.Phantom.entity"
            msg.base_data.actor_type = "airplane"
            msg.base_data.bounding_box.x = 10.0
            msg.base_data.bounding_box.y = 10.0
            msg.base_data.bounding_box.z = 10.0

            import random
            msg.attributes.max_velocity = random.uniform(0.0, 100.0)
            msg.attributes.max_acceleration = random.uniform(0.0, 100.0)
            msg.attributes.max_deceleration = random.uniform(0.0, 100.0)
            msg.attributes.max_roll_angle = random.uniform(0.0, 360.0)
            msg.attributes.max_pitch_angle = random.uniform(0.0, 360.0)
            msg.attributes.roll_rate = random.uniform(0.0, 100.0)
            msg.attributes.pitch_rate = random.uniform(0.0, 100.0)
            msg.attributes.yaw_rate_to_max_roll = random.uniform(0.0, 100.0)
            msg.attributes.limit_height = random.randint(0, 10000)
            msg.attributes.health_point = random.randint(0, 100)
            msg.attributes.damage_value = random.uniform(0.0, 100.0)
            msg.attributes.reconnaissance_radius = random.uniform(0.0, 100.0)
            msg.attributes.reconnaissance_angle = random.uniform(0.0, 360.0)
            msg.attributes.bombload = random.uniform(0.0, 100.0)
            msg.attributes.bomb_range = random.uniform(0.0, 1000.0)
            msg.attributes.bomb_velocity = random.uniform(0.0, 100.0)
            msg.attributes.bomb_cold_down_time = random.uniform(0.0, 60.0)

            msg.drone_swarm_kinematics_data.location.x = 10.0
            msg.drone_swarm_kinematics_data.location.y = 20.0
            msg.drone_swarm_kinematics_data.location.z = 30.0
            msg.drone_swarm_kinematics_data.rotation.pitch = 0.0
            msg.drone_swarm_kinematics_data.rotation.yaw = 0.0
            msg.drone_swarm_kinematics_data.rotation.roll = 0.0
            msg.drone_swarm_kinematics_data.velocity.linear.x = 0.0
            msg.drone_swarm_kinematics_data.velocity.linear.y = 0.0
            msg.drone_swarm_kinematics_data.velocity.linear.z = 0.0
            msg.drone_swarm_kinematics_data.acceleration.linear.x = 0.0
            msg.drone_swarm_kinematics_data.acceleration.linear.y = 0.0
            msg.drone_swarm_kinematics_data.acceleration.linear.z = 0.0
            msg.drone_swarm_kinematics_data.angular_velocity.linear.x = 0.0
            msg.drone_swarm_kinematics_data.angular_velocity.linear.y = 0.0
            msg.drone_swarm_kinematics_data.angular_velocity.linear.z = 0.0

            msg.load_data.remaining_projectiles = 10

            reconnaissance_data = BaseReconnaissanceData()
            for j in range(5):
                reconnaissance_data.targets.append(j)
            reconnaissance_data.num = 5
            msg.reconnaissance_data = reconnaissance_data

            interference_data = BaseInterferenceData()
            for j in range(5):
                target_info = BaseTargetInfo()
                target_info.id = j
                target_info.percent = 0.1 * j
                interference_data.targets.append(target_info)
            interference_data.num = 5
            msg.interference_data = interference_data

            drone_swarms.drone_swarms.append(msg)
            num += 1
        drone_swarms.header.timestamp_sec = time.time()
        drone_swarms.header.frame_id = "1"
        
        self.publisher_.publish(drone_swarms)
        self.get_logger().info('Publishing an ActorDroneSwarm message')

def main(args=None):
    rclpy.init(args=args)

    node = ProducerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
