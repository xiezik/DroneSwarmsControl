import rclpy
from rclpy.node import Node
from fq.msg import Header, ActorControlInfos,ActorControlInfo, ActorHitInfos ,ActorAirplanes, ActorBuildings, ActorDroneSwarms, ActorVehicles, ActorWarships, ActorZones
from geometry_msgs.msg import Point

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(ActorControlInfos, 'fq/drone_swarm_control_info', 10)
        self.subscription = self.create_subscription(
            ActorDroneSwarms,
            'fq/drone_swarm', 
            self.listener_callback,
            10)
        self.subscription
        self.drone_info = {}

    def listener_callback(self, dron_swarm_msg):
        # self.get_logger().info('我收到了: "%s"' % dron_swarm_msg)

        # 存储所有无人机的信息
        for i, dron_swarm in enumerate(dron_swarm_msg.drone_swarms):
            drone_id = dron_swarm.base_data.id
            self.drone_info[i] = {
                'drone_id': drone_id,
                'location': dron_swarm.drone_swarm_kinematics_data.location,
                'rotation': dron_swarm.drone_swarm_kinematics_data.rotation,
                'velocity': dron_swarm.drone_swarm_kinematics_data.velocity,
                'angular_velocity': dron_swarm.drone_swarm_kinematics_data.angular_velocity,
                'acceleration': dron_swarm.drone_swarm_kinematics_data.acceleration,
                'attributes': dron_swarm.attributes,
                'load_data': dron_swarm.load_data,
                'reconnaissance_data': dron_swarm.reconnaissance_data,
                'interference_data': dron_swarm.interference_data
            }

        dron_swarm_task_path = [[-2500.0, -305.0, 50.0], [-1700.0, -305.0, 50.0], [-1270.0, -90.0, 80.0]]      
        dron_swarm_test_heading = [0.0, 0.0, 0.0]  
        
        dron_swarms_control = ActorControlInfos()
        dron_swarms_control.header = dron_swarm_msg.header

        # self.get_logger().info('我收到了id: "%i"' % dron_swarm_msg.drone_swarms[0].base_data.id)
        # self.get_logger().info('我收到了actor_type: "%s"' % dron_swarm_msg.drone_swarms[0].base_data.actor_type)
        # self.get_logger().info('我收到了location: "%s"' % dron_swarm_msg.drone_swarms[0].drone_swarm_kinematics_data)
        '''
         fq.msg.ActorDroneSwarm(
                    base_data=fq.msg.BaseBaseData(
                                id=3, 
                                health_point=20, 
                                type_id='airplane.mini3.entity', 
                                actor_type='EntityAirplane', 
                                bounding_box=fq.msg.BaseBoundingBox(x=0.19449999928474426, y=0.16050000488758087, z=0.029999999329447746), b_target=False), 
                    attributes=fq.msg.BasePlaneAttr(
                                airfoil_type='Rotor', 
                                load_type='SuicideRotor', 
                                min_velocity=0.0, 
                                max_velocity=10.0, 
                                max_acceleration=2.0, 
                                max_deceleration=2.0, 
                                max_roll_angle=0.0, 
                                max_pitch_angle=0.0, 
                                roll_rate=0.0, 
                                pitch_rate=0.0, 
                                yaw_rate_to_max_roll=20.0, 
                                limit_height=60.0, 
                                health_point=20, 
                                damage_value=20, 
                                reconnaissance_radius=20.0, 
                                reconnaissance_angle=0.0, 
                                bombload=0, 
                                bomb_range=0.0, 
                                bomb_velocity=0.0, 
                                bomb_cold_down_time=0.0), 
                    drone_swarm_kinematics_data=fq.msg.BaseAirplaneKinematicsData(
                                location=geometry_msgs.msg.Point(x=-3000.0, y=0.0, z=10.0), 
                                rotation=fq.msg.BaseOrientation3D(roll=0.0, pitch=0.0, yaw=0.0), 
                                velocity=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)), 
                                angular_velocity=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)), 
                                acceleration=geometry_msgs.msg.Accel(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))), 
                                load_data=fq.msg.BaseLoadData(remaining_projectiles=0), 
                                reconnaissance_data=fq.msg.BaseReconnaissanceData(targets=[], num=0), 
                                interference_data=fq.msg.BaseInterferenceData(targets=[], num=0)
                    )
        '''


        # for dron_swarm in dron_swarm_msg.drone_swarms:
        #     _id = dron_swarm.base_data.id
        #     dron_swarm_control = ActorControlInfo()
        #     dron_swarm_control.id = _id
        #     dron_swarm_control.target_positions = [Point(x=i[0], y=i[1], z=min(dron_swarm.drone_swarm_kinematics_data.location.z, dron_swarm.attributes.limit_height)) for i in dron_swarm_test_path]
        #     dron_swarm_control.target_velocity = dron_swarm.attributes.max_velocity
        #     dron_swarm_control.max_velocity = dron_swarm.attributes.max_velocity
        #     dron_swarm_control.target_headings = dron_swarm_test_heading
        #     dron_swarms_control.control_info.append(dron_swarm_control)

        # 通过id获取某个无人机的信息
        control_id = 801
        dron_swarm = dron_swarm_msg.drone_swarms[control_id]
        # self.get_logger().info('我收到了actor_type: "%s"' % dron_swarm_msg.drone_swarms[100].base_data.actor_type)
        self.get_logger().info('我收到了load_type: "%s"' % dron_swarm_msg.drone_swarms[control_id].attributes.load_type)

        for i in range(500 , 599):
            dron_swarm_control = ActorControlInfo()
            dron_swarm_control.id = self.drone_info[i]['drone_id']
            dron_swarm_control.target_positions = [Point(
            x=self.drone_info[i]['location'].x+5000.0, 
            y=self.drone_info[i]['location'].y+0.0, 
            z=min(self.drone_info[i]['location'].z, self.drone_info[i]['attributes'].limit_height)) 
            for _ in range(len(dron_swarm_task_path))]
            
            dron_swarm_control.target_velocity = self.drone_info[i]['attributes'].max_velocity
            dron_swarm_control.max_velocity = self.drone_info[i]['attributes'].max_velocity
            dron_swarm_control.target_headings = dron_swarm_test_heading
            dron_swarms_control.control_info.append(dron_swarm_control)

        # dron_swarm_control = ActorControlInfo()
        # dron_swarm_control.id = self.drone_info[control_id]['drone_id']
        # dron_swarm_control.target_positions = [Point(x=i[0], y=i[1], z=min(self.drone_info[control_id]['location'].z, self.drone_info[control_id]['attributes'].limit_height)) for i in dron_swarm_test_path]
        # dron_swarm_control.target_velocity = self.drone_info[control_id]['attributes'].max_velocity
        # dron_swarm_control.max_velocity = self.drone_info[control_id]['attributes'].max_velocity
        # dron_swarm_control.target_headings = dron_swarm_test_heading
        # self.dron_swarms_control.control_info.append(dron_swarm_control)
        self.publisher_.publish(dron_swarms_control)
        # 这里为了避免多次控制，可删除
        exit()
def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
