ROS Inception Test:

ROS takes care of a variety of requirements in a robot: multithreading, messaging and callbacks. The publisher subscriber methods works fairly well, but ROS2 services can have a problem. In the classic way of programming them, the ecosystem seems to expect to return the answer right away.
In OOP we would expect that we don't need to know what happens downstream. in the case of an asynchronous service call, our client wants to call the service and expects to get the answer in return, which will then trigger the client's callback function. We actually don't really care how the server obtains the required information and technically we don't even care how long it takes, since we have a callback in place. Yet, unfortunately this doesn't always seem to work. 

lets assume 3 nodes: Client -> Service1 -> Service2
In case our service1 calls another service2 to fullfil the request of its client, some futures seem not to actually wait for the done-callback, but send an empty message back within the next spinning cycle.
So in this scenario "Client" creates a future object and calls "Service1". "Service1" creates also a future and calls (in its programming loop) "service2" (so service1 is also a client for service2).
"Service1" should in theory wait for the future to be filled by the response of "service2" and then send that result back to the original client. But with some functions that doesn't seem to work. 

This code explores this scenario. I actually added a 4th node just to amplify this problem and make sure that it will work with multiple service inceptions. If you for example run this code in 4 different terminals (one for each node) then you will see that the messages from Node_four are not arriving in Node_one, which shows that the services return empty messages even though they should technically wait for their future to be filled by an asynchronous service call first. In summary: you can't write services without knowing the bigger picture of the code. Knowing of this issue it is worth to look into executors and callback groups and what they can do for you in this case. 

These are the relevant files that need to be manipulated for this code
![Screenshot from 2024-02-28 20-52-44](https://github.com/DavidBierbrauer/Testfunctions/assets/47460151/ecbb6097-fb93-46f0-8a61-38224beb3468)

and a folder hierarchy is usually in place, especially when an editor like Visual Studio Code is used
![Screenshot from 2024-02-28 20-51-52](https://github.com/DavidBierbrauer/Testfunctions/assets/47460151/3b5136f3-670c-4df9-850f-be45460696e1)

if we use terminator we can display can start and display all nodes quickly via the cammand terminals
![Screenshot from 2024-03-01 01-08-19](https://github.com/DavidBierbrauer/Testfunctions/assets/47460151/cba4e92e-ac8e-41ef-99c9-f5f533a94f92)

If you go into the code, then you see that each node writes sends their service call with a message that contains a string with the timestamp and that they called the next node. Each node picks up that message and for their own servic call they attach the previous message after an underscore_. In node_four it is the easiest to see.
one:0_two:xxxxxx_three:xxxxxxx_four:xxxxxxx. This shows that the messages are correctly forwarded.
As we can see in the code, for the response every function is supposed to take the previous response and add their _b2 (or _b1,_b3, _b4), but this doesn't happen as expected. the message from node 4 is not all the way backpropagated in the same cycle. The service response is therefore resolved before the future from the inception service is actually filled. This is good to know, since therefore we can not start a service within a service without very very explicitely telling the system to wait with the response until the future is filled. like mentioned before, at this point it is useful to look into executors and callback groups and what they can do for us in this case.
![Screenshot from 2024-03-01 03-21-47](https://github.com/DavidBierbrauer/Testfunctions/assets/47460151/335e5b99-a915-4492-be21-ef4e4f813de5)

