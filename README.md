# ROS2 Tutorial 2.2: Services & Clients

This is Part 2 in a series of ROS2 Tutorials:
1. [Publishers & Subscribers](https://github.com/Woolfrey/tutorial_publisher_subscriber)
2. Services
     1. [Defining a Service](https://github.com/Woolfrey/tutorial_service_definition)
     2. [Creating a Service & Client](https://github.com/Woolfrey/tutorial_service_client)
4. Actions
     1. [Defining an Action](https://github.com/Woolfrey/tutorial_action_definition)
     2. [Creating an Action Server & Client](https://github.com/Woolfrey/tutorial_action_server)
        
## Contents
- [What Are They?](#what-are-they)
- [1. Writing a Server Node](#writing-a-server-node)
- [2. Writing a Client Node](#writing-a-client-node)

## What Are They?

A `service` in ROS2 is a structured message type that defines a `request` data type, and a `return` data type. Is is contained in an `.srv` file with the following structure:
```
# Request
datatype request_name
---
# Response
datatype response_name
```
This defines how a client node can communicate with a service node. The client makes a request of the specified data type. The server does something with that information, and returns the eponse data type.

A `service` differs from the `publisher` and `subscriber` paradigm in that the latter is suited to streaming data. A publisher node will make data publicly available on the ROS2 network that any number of subscribers may access and make use of. In contrast, a service is a direction communication between the server and client.

We can think of the publisher and subscriber model like a newsagency, where different magazines and journals are made publicly available that customers can subscribe to based on relevant interests.

![image](https://github.com/Woolfrey/tutorial_service_definition/assets/62581255/5ee507a5-65cb-4eac-9466-4b4e3efc96e5)

The service and client model is more like sending a letter directly to someone, who will then write back directly to you with the requested information.

<img src="https://github.com/Woolfrey/tutorial_service_definition/assets/62581255/6fa5991a-1272-4ddd-960a-dee4ec8a3217" alt="img" width="500" height="auto">

## 1. Writing a Service Node

### :mag: The Code Explained

:arrow_backward: [Go back.](#ros2-tutorial-22-service--clients)

## 2. Writing a Client Node

### :mag: The Code Explained

:arrow_backward: [Go back.](#ros2-tutorial-22-service--clients)
