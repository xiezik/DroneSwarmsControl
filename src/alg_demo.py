import rclpy
from rclpy.node import Node
from fq.msg import Header, ActorControlInfos,ActorControlInfo, ActorHitInfos ,ActorAirplanes, ActorBuildings, ActorDroneSwarms, ActorVehicles, ActorWarships, ActorZones
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import threading
import time
import math

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('my_node')
         # 定义可视化发布器
        self.marker_publisher = self.create_publisher(MarkerArray, 'fq/visualization_markers', 10)

        self.drone_swarm_control_publisher = self.create_publisher(ActorControlInfos, 'fq/drone_swarm_control_info', 10)
        self.drone_swarm_sub = self.create_subscription(
            ActorDroneSwarms,
            'fq/drone_swarm', 
            self.drone_swarm_listener_callback,
            10)
        self.warship_sub = self.create_subscription(
            ActorWarships,
            'fq/warship', 
            self.warship_listener_callback,
            10)
        
        self.last_timestamp = None  # 上一个时间戳
        self.time_delta = 0  # 计算时间间隔
        self.first_ship_received = False  # 是否收到第一个舰船信息

        self.warship_sub    
        self.drone_swarm_sub
        self.drone_info = {}
        self.warship_info = {}
        self.warship_detected_info = {}
        self.first_ship_received = False  # 新增标志变量
        self.first_ship_position = None  # 用于存储第一个舰船的位置
        
        self.dron_swarms_control = ActorControlInfos()

        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

        # 启动可视化线程
        self.visualization_thread = threading.Thread(target=self.visualization_loop)
        self.visualization_thread.start()

    def control_loop(self):
        dron_swarm_task_path = [[8000.0, -305.0, 60.0], [-1700.0, -305.0, 50.0], [-1270.0, -90.0, 80.0]] 
        dron_swarm_task_heading = [0.0, 0.0, 0.0] # 三个路径点的yaw角度
        while rclpy.ok():
            # 等待无人机信息
            # print(self.drone_info)
            if not self.drone_info:
                time.sleep(1)
                continue
            # 控制Patrol机群（ID范围：500-599）
            
            self.swarm_control(500, 599, dron_swarm_task_path, dron_swarm_task_heading)
            
            # 控制SuicideRotor机群（ID范围：0-499）
            self.swarm_control(0, 499, dron_swarm_task_path, dron_swarm_task_heading)

            # 控制SuicideFixed机群（ID范围：600-899）
            self.swarm_control(600, 899, dron_swarm_task_path, dron_swarm_task_heading)

            # 控制Bomber机群（ID范围：900-999）
            self.swarm_control(900, 999, dron_swarm_task_path, dron_swarm_task_heading)
            self.drone_swarm_control_publisher.publish(self.dron_swarms_control)
            self.dron_swarms_control.control_info=[]

            break
        
        while rclpy.ok():
            if self.warship_info:
                for i in range(600 , 899):
                    dron_swarm_control = ActorControlInfo()
                    dron_swarm_control.id = self.drone_info[i]['drone_id']
                    dron_swarm_control.target_positions = [Point(
                    x=self.warship_info[0]['location'].x, 
                    y=self.warship_info[0]['location'].y, 
                    z=min(self.warship_info[0]['location'].z, self.drone_info[i]['attributes'].limit_height)) 
                    for _ in range(len(dron_swarm_task_path))]
                    
                    dron_swarm_control.target_velocity = self.drone_info[i]['attributes'].max_velocity
                    dron_swarm_control.max_velocity = self.drone_info[i]['attributes'].max_velocity
                    dron_swarm_control.target_headings = dron_swarm_task_heading
                    self.dron_swarms_control.control_info.append(dron_swarm_control)
                self.drone_swarm_control_publisher.publish(self.dron_swarms_control)
                self.dron_swarms_control.control_info=[]
                print('侦查到舰船，发送攻击控制信息')
                break
            else:
                time.sleep(1)
                print('等待舰船信息')
                # print(self.warship_info)
                continue
    def visualization_loop(self):
        while rclpy.ok():
            time.sleep(1)  # 延时模拟刷新时间
            self.publish_visualization()
            # self.visualize_warships()


    def publish_visualization(self):
        marker_array = MarkerArray()

        # 1. 发布无人机位置
        for drone_id, drone_data in self.drone_info.items():
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = drone_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = drone_data['location']
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            if drone_data['attributes'].load_type == 'SuicideRotor':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif drone_data['attributes'].load_type == 'PatrolRotor':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif drone_data['attributes'].load_type == 'SuicideFixed':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif drone_data['attributes'].load_type == 'Bomber':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        # 2. 发布舰船位置
        for ship_id, ship_data in self.warship_info.items():
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 1000 + ship_id  # 让舰船的 ID 从 1000 开始，避免与无人机重复
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = ship_data['location']
            marker.scale.x = marker.scale.y = marker.scale.z = 5.0  # 适当放大
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  # 蓝色
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        # 发布所有Marker
        self.marker_publisher.publish(marker_array)


    def swarm_control(self, start_id, end_id, dron_swarm_task_path, dron_swarm_task_heading, is_attack_mode=False):
        """
        控制蜂群的运动，适应不同类型的机群。
        
        :param start_id: 起始ID
        :param end_id: 结束ID
        :param dron_swarm_task_path: 任务路径
        :param dron_swarm_task_heading: 任务航向
        :param is_attack_mode: 是否进入攻击模式，若为True，则目标为舰船
        """
        for i in range(start_id, end_id):
            dron_swarm_control = ActorControlInfo()
            dron_swarm_control.id = self.drone_info[i]['drone_id']

            # 如果是攻击模式，目标为舰船，否则目标为默认位置
            if is_attack_mode and self.warship_info:
                target_location = self.warship_info[0]['location']
            else:
                target_location = Point(
                    x=self.drone_info[i]['location'].x + 5000.0, 
                    y=self.drone_info[i]['location'].y + 0.0, 
                    z=min(self.drone_info[i]['location'].z, self.drone_info[i]['attributes'].limit_height)
                )

            # 设置目标位置
            dron_swarm_control.target_positions = [target_location for _ in range(len(dron_swarm_task_path))]
            
            # 设置目标速度、最大速度、航向等
            dron_swarm_control.target_velocity = self.drone_info[i]['attributes'].max_velocity
            dron_swarm_control.max_velocity = self.drone_info[i]['attributes'].max_velocity
            dron_swarm_control.target_headings = dron_swarm_task_heading
            
            # 添加到控制指令中
            self.dron_swarms_control.control_info.append(dron_swarm_control)



    def warship_listener_callback(self, warship_msg):

        current_time = warship_msg.header.timestamp_sec  # 获取当前时间戳

        # 更新已探测舰船的信息
        for warship in warship_msg.warships:
            ship_id = warship.base_data.id

            # 判断舰船是否在已探测的舰船列表中
            if ship_id not in self.warship_info:
                # 如果是新探测到的舰船，记录其初始位置和其他信息
                self.warship_info[ship_id] = {
                    'ship_id': ship_id,
                    'location': warship.kinematics_data.location,
                    'rotation': warship.kinematics_data.rotation,
                    'health_point': warship.base_data.health_point,
                    'velocity': 5.0,  # 固定速度
                    'ship_type': warship.base_data.type_id,
                    'bounding_box': warship.base_data.bounding_box,
                    'last_update_time': current_time  # 记录上次更新的时间
                }
            else:

                # 更新舰船位置
                self.warship_info[ship_id]['location'].x = warship.kinematics_data.location.x
                self.warship_info[ship_id]['location'].y = warship.kinematics_data.location.y
                self.warship_info[ship_id]['health_point'].y = warship.base_data.health_point


                # 更新最后更新时间戳
                self.warship_info[ship_id]['last_update_time'] = current_time

        # 在没有探测到某些舰船时，继续使用已知信息进行位置更新
        for ship_id, ship_data in self.warship_info.items():
            # 如果当前舰船ID在当前消息中没有出现，则使用它的速度和上次的时间戳更新位置
            if ship_id not in [warship.base_data.id for warship in warship_msg.warships]:
                # 获取时间差（秒）
                time_diff = (current_time - ship_data['last_update_time'])
                # 使用速度和时间差来更新位置
                velocity = ship_data['velocity']
                delta_x = velocity * time_diff * math.cos(ship_data['rotation'].yaw)
                delta_y = velocity * time_diff * math.sin(ship_data['rotation'].yaw)

                # 更新舰船位置
                ship_data['location'].x += delta_x
                ship_data['location'].y += delta_y

                # 更新最后更新时间戳
                ship_data['last_update_time'] = current_time



    def update_fleet_position(self, time_delta):
        if not self.first_ship_received:  # 如果没有探测到任何舰船，返回
            return

        # 找到第一个探测到的舰船
        detected_ships = [warship for warship in self.warship_info.values()]

        if not detected_ships:
            return

        # 计算舰队中心点位置
        if any(warship['ship_type'] == 'militaryship.cvn_76' for warship in detected_ships):
            # 以cvn_76类型舰船为中心
            center_ship = next(warship for warship in detected_ships if warship['ship_type'] == 'militaryship.cvn_76')
            fleet_center = center_ship['location']
        else:
            # 如果没有cvn_76舰船，使用所有探测到的舰船的中点作为中心
            x_sum = sum(warship['location'].x for warship in detected_ships)
            y_sum = sum(warship['location'].y for warship in detected_ships)
            z_sum = sum(warship['location'].z for warship in detected_ships)
            fleet_center = Point(x=x_sum / len(detected_ships), y=y_sum / len(detected_ships), z=z_sum / len(detected_ships))

        # 计算舰队的航向（假设平行航行，航向由第一个探测到的舰船的rotation决定）
        first_ship_rotation = detected_ships[0]['rotation']
        fleet_heading = first_ship_rotation.yaw  # 使用第一个舰船的yaw作为航向

        # 更新舰队的模型信息
        self.get_logger().info(f'Updated fleet position to center {fleet_center} with heading {fleet_heading}')

        # 此处可以继续处理舰队模型的进一步更新和预测打击逻辑


    def drone_swarm_listener_callback(self, dron_swarm_msg):
        # self.get_logger().info('我收到了: "%s"' % dron_swarm_msg)
        self.dron_swarms_control.header = dron_swarm_msg.header

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
        
        dron_swarms_control = ActorControlInfos()
        dron_swarms_control.header = dron_swarm_msg.header

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


def main(args=None):
    rclpy.init(args=args)
    drone_conrol_node = DroneControlNode()
    rclpy.spin(drone_conrol_node)
    drone_conrol_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
