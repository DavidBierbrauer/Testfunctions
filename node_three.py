#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from haive_interfaces.srv import Inception
import time

class three_Node(Node): # name of what it does
    def __init__(self):
        self.name = "three_node"
        super().__init__(self.name) # don't use "node" in the name, because it will be redundant
        self.server_ = self.create_service(Inception,"call_three",self.callback_called_by_two)
        self.received = ""
        self.get_logger().info("three_Node has been started...")


    # here we will see again that the service call self.call_four_node(request.time,new_message)
    # will not be resolved before the service server callback_called_by_two(self,request,response) will answer to node two
    # if we comment out self.received = "", then we don't reset it and will see that the next spins will actually slowly carry the message forward
    # this is of course only because we use another variable to safe the previous information
    def callback_called_by_two(self,request,response):
        self.received = ""
        duration=time.time_ns()-request.time
        new_message = request.message+("_three:{}".format(duration))
        self.call_four_node(request.time,new_message)
        response.feedback = self.received
        return response # it needs a return (if the service has a return), otherwise the code throws an error

    ### standard client ###
    def call_four_node(self, time,message):
        client = self.create_client(Inception, "call_four")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server_four...")

        request = Inception.Request()
        request.time = time
        request.message = message
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_four, call =(time,message)))
         
    def callback_call_four(self, future, call):
        try:
            response4 = future.result()
            response4.feedback = response4.feedback + "_b3"
            self.get_logger().info("call_four at {}:{}, feedback:{}".format(call[0], call[1],response4.feedback))
            self.received = response4.feedback
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

# main can always be the same for every node!! 
def main(args=None):
    rclpy.init(args=args)
    node = three_Node() # MODIFY NAME
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
     main()
