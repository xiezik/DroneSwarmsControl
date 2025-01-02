import time
from api import FqTopic, API
from fq.msg import ActorDroneSwarms, ActorControlInfos, ActorControlInfo

def demo(args=None):
    API.init(args)

    # 1.注册回调
    def drone_swarm_callback(msg):
        drone_swarm_control = ActorControlInfos()
        drone_swarm_control.header = msg.header
        # for drone_swarm in msg.drone_swarms:
        #     print('base_data', drone_swarm.base_data)
        #     print('attributes', drone_swarm.attributes)
        #     print('drone_swarm_kinematics_data', drone_swarm.drone_swarm_kinematics_data)
        #     print('load_data', drone_swarm.load_data)
        #     print('reconnaissance_data', drone_swarm.reconnaissance_data)
        #     print('interference_data', drone_swarm.interference_data)
        #     print('#'*50)
            # TODO: Fill in the fields of the control message based on the received message
        
        API.get_logger().info(f'drone_swarm_callback received dron swarm param')

    API.register_callback(FqTopic.drone_swarm_param, drone_swarm_callback)

    # 2.主动获取
    while True:
        drone_swarm_msg = API.next_drone_swarm_param()
        # if drone_swarm_msg is not None:
            # drone_swarm_callback(drone_swarm_msg)
        time.sleep(5)
        API.get_logger().info(f'received dron swarm')

        drone_swarm_control = ActorControlInfos()
        drone_swarm_control.header = drone_swarm_msg.header
        API.publish_control_command(drone_swarm_control)
        API.get_logger().info(f'published dron swarm control')
    API.destory()

def main(args=None):
    demo(args)

if __name__ == '__main__':
    main()
