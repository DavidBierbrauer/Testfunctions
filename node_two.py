#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from haive_interfaces.srv import Inception
import time

# we want to log the received message, otherwise this is again just a minal ROS2 node
class two_Node(Node): # name of what it does
    def __init__(self):
        self.name = "two_node"
        super().__init__(self.name) # don't use "node" in the name, because it will be redundant
        self.server_ = self.create_service(Inception,"call_two",self.callback_called_by_one)
        self.received = ""
        self.get_logger().info("two_Node has been started...")

    # the received message needs a reset, otherwise we will accumulate them.
    # The self.received class variable will be updated during the inception response
    # we also see that in our callback from node_one we will actually call node_three with a service
    def callback_called_by_one(self,request,response):
        self.received = ""
        duration=time.time_ns()-request.time
        new_message = request.message+("_two:{}".format(duration))
        self.call_three_node(request.time,new_message)
        response.feedback = self.received
        return response # it needs a return (if the service has a return), otherwise the code throws an error

    ### standard client ###
    def call_three_node(self, time,message):
        client = self.create_client(Inception, "call_three")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server_three...")

        request = Inception.Request()
        request.time = time
        request.message = message
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_three, call =(time,message)))
    
    # the callback from node three 
    def callback_call_three(self, future, call):
        try:
            response3 = future.result()
            response3.feedback = response3.feedback + "_b2"
            self.get_logger().info("call_three at {}:{}, feedback:{}".format(call[0], call[1],response3.feedback))
            self.received = response3.feedback
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

# main can always be the same for every node!! 
def main(args=None):
    rclpy.init(args=args)
    node = two_Node() # MODIFY NAME
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
     main()
