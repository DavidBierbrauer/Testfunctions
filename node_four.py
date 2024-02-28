#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from haive_interfaces.srv import Inception
import time

class four_Node(Node): # name of what it does
    def __init__(self):
        self.name = "four_node"
        super().__init__(self.name) # don't use "node" in the name, because it will be redundant
        self.server_ = self.create_service(Inception,"call_four",self.callback_called_by_three)
        self.get_logger().info("four_Node has been started...")

    def callback_called_by_three(self,request,response):
        duration=time.time_ns()-request.time
        new_message = request.message+("_four:{}".format(duration))
        response.feedback = new_message
        time.sleep(3)
        print("send response {}".format(response.feedback))
        
        return response # it needs a return (if the service has a return), otherwise the code throws an error

# main can always be the same for every node!! 
def main(args=None):
    rclpy.init(args=args)
    node = four_Node() # MODIFY NAME
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
     main()