# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from api import ChuzhiTopic, API
from chuzhi.msg import ControlCommand, HitInfo

def demo(args=None):

    API.init(args)
    
    def mychassis_callback(msg):
        API.get_logger().info(f'mychassis_callback received Chassis: {msg}')
        print(msg)
        
    def mychassis_callback2(msg):
        API.get_logger().info(f'mychassis_callback2 received pose: {msg}')
        print(msg)
    
    API.register_callback(ChuzhiTopic.chassis, mychassis_callback)
    API.register_callback(ChuzhiTopic.pose, mychassis_callback2)
    
    while True:
        chassis = API.next_chassis()
        pose = API.next_pose()
        obstacles = API.next_obstacles()
        
        # alg process
        # API.get_logger().info(f'received Chassis: {chassis}')
        # API.get_logger().info(f'received pose: {pose}')
        # API.get_logger().info(f'received obstacles: {obstacles}')
        
        min_speed = 2
        max_speed = 4
        control = ControlCommand()
        control.throttle = 0.0
        control.brake = 100.0
        control.steer = 0.0
        
        # if pose.pose.position.x > -400.0:
        #     control.brake = 1.0
        # elif chassis.speed_mps < min_speed:
        #     control.throttle = 0.6
        # elif chassis.speed_mps > max_speed:
        #     control.brake = 0.5
        # else:
        #     pass
        
        control.hand_brake = False
        API.publish_control_command(control)
        
        hit = HitInfo()
        hit.should_hit = False
        
        blue1 = obstacles.obstacles[0]
        blue2 = obstacles.obstacles[1]
        blue = blue1 if blue1.id > blue2.id else blue2
        # print(blue.id, blue.position.y, pose.pose.position.x)
        if blue.position.y < -429.0 and blue.position.y > -430.1:
            hit.should_hit = True
            API.get_logger().info(f'blue.position.y: {blue.position.y}')
        # if blue.position.y < -439.0 and blue.position.y > -440.1 and pose.pose.position.x > -500.2389:
        #     hit.should_hit = True
        if pose.pose.position.x > -430.439636:
            hit.should_hit = False
        
        API.publish_hit_info(hit)


    API.destory()

def main(args=None):
    
    demo(args)
    
    # rclpy.init(args=args)
    # client = SimMessageNode()
    # rclpy.spin(client)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # client.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
