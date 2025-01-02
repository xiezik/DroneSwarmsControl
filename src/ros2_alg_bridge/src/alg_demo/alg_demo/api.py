import threading
from queue import Queue
from queue import Empty

from enum import Enum


import sys
import glob
sys.path.append(glob.glob('../'))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
from fq.msg import Header, ActorControlInfos, ActorHitInfo ,ActorAirplanes, ActorBuildings, ActorDroneSwarms, ActorVehicles, ActorWarships, ActorZones


'''
    fq ros2 topics
'''

class FqTopic(Enum):

    # 蜂群参数
    drone_swarm_param = "fq/drone_swarm"

    # 蜂群控制返回值
    drone_swarm_control = "fq/drone_swarm_control"

    # 蜂群打击返回值
    drone_swarm_hit_info = "fq/drone_swarm_hit_info"

    # 建筑信息
    building_param = "fq/building"

    # 无人机参数
    air_plane_param = "fq/air_plane"

    # 车辆参数
    vehicle_param = "fq/vehicle"

    # 无人船参数
    warship_param = "fq/warship"

    # 特殊区域参数
    zone_param = "fq/zone"

FqTopic2Message = {
    FqTopic.drone_swarm_param: ActorDroneSwarms,
    FqTopic.drone_swarm_control: ActorControlInfos,
    FqTopic.drone_swarm_hit_info: ActorHitInfo,
    FqTopic.building_param: ActorBuildings,
    FqTopic.air_plane_param: ActorAirplanes,
    FqTopic.vehicle_param: ActorVehicles,
    FqTopic.warship_param: ActorWarships,
    FqTopic.zone_param: ActorZones
} 


class SimMessageNode(Node):

    class TopicSubscriptionHandler(object):
        def __init__(self, ros2_node, topic:FqTopic):
            super().__init__()
            self._ros2_node = ros2_node
            self._topic = topic
            self._buffer = Queue()
            self._callback_list = []
            
            self._subscription = ros2_node.create_subscription(
                FqTopic2Message.get(topic),
                topic.value,
                self._callback,
                10)
            
        def next_data(self):
            return self._buffer.get(True)
            
        def _callback(self, msg):
            self._buffer.put(msg)
            
            # TODO: 应改为异步触发，否则容易阻塞
            for call in self._callback_list:
                call(msg)
        
        def register_callback(self, callback):
            self._callback_list.append(callback)
            
        def clear_callback(self):
            self._callback_list.clear()
        
    def __init__(self):
        super().__init__('SimMessageNode')
        self._subscription_handler = {
            FqTopic.drone_swarm_param: SimMessageNode.TopicSubscriptionHandler(self, FqTopic.drone_swarm_param),
            FqTopic.building_param: SimMessageNode.TopicSubscriptionHandler(self, FqTopic.building_param),
            FqTopic.air_plane_param: SimMessageNode.TopicSubscriptionHandler(self, FqTopic.air_plane_param),
            FqTopic.vehicle_param: SimMessageNode.TopicSubscriptionHandler(self, FqTopic.vehicle_param),
            FqTopic.warship_param: SimMessageNode.TopicSubscriptionHandler(self, FqTopic.warship_param),
            FqTopic.zone_param: SimMessageNode.TopicSubscriptionHandler(self, FqTopic.zone_param),
        }
        self._control_command_publisher = self.create_publisher(
            FqTopic2Message.get(FqTopic.drone_swarm_control),
            FqTopic.drone_swarm_control.value,
            10
        )

        self._hit_publisher = self.create_publisher(
            FqTopic2Message.get(FqTopic.drone_swarm_hit_info),
            FqTopic.drone_swarm_hit_info.value,
            10
        )
        

    '''
        callback 顺序阻塞式执行，不应注册耗时操作，执行顺序与注册顺序相关
    '''
    def register_callback(self, topic:FqTopic, callback):
        if topic in self._subscription_handler:
            self._subscription_handler[topic].register_callback(callback)
        else:
            self.get_logger().error(f'invalid topic callback: {FqTopic}')
        
    def publish_control_command(self, msg):
        self._control_command_publisher.publish(msg)
        self.get_logger().info(f'send control command: {msg}')
        
    def publish_hit_info(self, msg):
        self._hit_publisher.publish(msg)
        self.get_logger().info(f'send hit info: {msg}')
                
    def next_data(self, topic:FqTopic):
        if topic in self._subscription_handler:
            return self._subscription_handler[topic].next_data()
        else:
            self.get_logger().error(f'invalid topic callback: {topic}')
            return None

class API(object):
    _start = False
    _sim_node = None
    _spin_thread = None

    @staticmethod
    def init(args=None):
        try:
            rclpy.init(args=args)
            API._sim_node = SimMessageNode()
            API._spin_thread = threading.Thread(target=rclpy.spin, args=(API._sim_node,))
            API._spin_thread.start()
            API._start = True   
            API.get_logger().info(f'API init success~~')

        except Exception as e:
            print(e)
            API.destory()

    @staticmethod
    def destory():
        try:
            if API._start is True:
                API._sim_node.destroy_node()
                API.get_logger().info(f'API destory success~')
                API._sim_node = None
                rclpy.shutdown()
                API._spin_thread.join()
                API._spin_thread = None
                API._start = False
        except Exception as e:
            print(e)


    '''
        callback 顺序阻塞式执行，不应注册耗时操作，执行顺序与注册顺序相关
    '''
    @staticmethod
    def subscribe(topic:FqTopic, callback):
        API._sim_node.register_callback(topic, callback)

    @staticmethod   
    def register_callback(topic:FqTopic, callback):
        API._sim_node.register_callback(topic, callback)
    
    @staticmethod
    def publish_control_command( msg):
        API._sim_node.publish_control_command(msg)
        
    @staticmethod
    def publish_hit_info( msg):
        API._sim_node.publish_hit_info(msg)
    
    @staticmethod   
    def next_data(topic:FqTopic):
        return API._sim_node.next_data(topic)

    @staticmethod
    def next_drone_swarm_param():
        return API.next_data(FqTopic.drone_swarm_param)
    
    @staticmethod
    def next_building_param():
        return API.next_data(FqTopic.building_param)
    
    @staticmethod
    def next_vehicle_param():
        return API.next_data(FqTopic.vehicle_param)
    
    @staticmethod
    def next_zone_param():
        return API.next_data(FqTopic.zone_param)
    
    @staticmethod
    def next_airplane_param():
        return API.next_data(FqTopic.air_plane_param)
    
    @staticmethod
    def next_warship_param():
        return API.next_data(FqTopic.warship_param)
    
    @staticmethod
    def get_logger():
        return API._sim_node.get_logger()

