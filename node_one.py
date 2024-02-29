#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from haive_interfaces.srv import Inception
import time

# this node is pretty straight forward, we create a timer that calls node_two
class one_Node(Node): 
    def __init__(self):
        self.name = "one_node"
        super().__init__(self.name) 
        self.create_timer(5, self.timer_callback) # create a callback function called by timer
        self.get_logger().info("one_Node has been started...")

    # we send the current time and a message one:0 
    # indicating that node one send its message at time 0 (start of the inception call)
    def timer_callback(self):
        self.call_two_node(time.time_ns(),"one:0")

    # also very straight forward. We create a future for an async call and add a callback when done
    # the partial function is a little trick that allows us to add additional args to the call
    def call_two_node(self, time,message):
        client = self.create_client(Inception, "call_two")
        print("requesting")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server_two...")

        request = Inception.Request()
        request.time = time
        request.message = message
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_two, call =(time,message)))

    # print the response from the service call of node_two, together with our two call partials args
    def callback_call_two(self, future, call):
        try:
            response = future.result()
            self.get_logger().info("call_two at {}:{}, feedback:{}".format(call[0], call[1],response.feedback))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

# main can always be the same for every node!! 
def main(args=None):
    rclpy.init(args=args)
    node = one_Node() # MODIFY NAME
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
     main()
