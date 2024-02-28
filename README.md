HaiveEye:

A test script to test the ballpark of accuracy for interaction sensors,
based on a simple setup of a high accuracy ball screw with encoder controlled by an arduino with a simple firmware,
a high accuracy linear gauge for reference
and a sensor (light, hall etc...) that needs to be evaluated

ROS Inception Test:

ROS2 services can have a problem. In the classic way of programming them, the ecosystem seems to expect to return the answer right away.
In OOP we would expect that we don't need to know what happens downstream. in the case of an asynchronous service call, our client wants to call the service
and expects to get the answer in return, which will then trigger the client's callback function. We actually don't really care how the server obtains the required information
and technically we don't even care how long it takes, since we have a callback in place. Yet, unfortunately this doesn't always seem to work. 
lets assume 3 nodes: Client -> Service1 -> Service2
In case our service1 calls another service2 to fullfil the request of its client, some futures seem not to actually wait for the done-callback, but send an empty message back within the next spinning cycle.
So in this scenario client creates a future object and calls Service1. Service1 creates also a future and calls (in its programming loop) service2 (so service1 is also a client for service2).
Service1 should in theory wait for the future to be filled by the response of service2 and then send that result back to the original client. But with some functions that doesn't seem to work. 
This code explores this scenario and tries to fix it. I actually added a 4th node just to amplify this problem and make sure that it will work with multiple service inceptions.

These are the relevant files that need to be manipulated
![Screenshot from 2024-02-28 20-52-44](https://github.com/DavidBierbrauer/Testfunctions/assets/47460151/ecbb6097-fb93-46f0-8a61-38224beb3468)

and a folder hierarchy is usually in place, especially when an editor like Visual Studio Code is used
![Screenshot from 2024-02-28 20-51-52](https://github.com/DavidBierbrauer/Testfunctions/assets/47460151/3b5136f3-670c-4df9-850f-be45460696e1)
