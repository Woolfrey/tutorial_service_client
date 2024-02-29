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
- [1. Writing a Service Node](#1-writing-a-service-node)
- [2. Writing a Client Node](#2-writing-a-client-node)

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

:arrow_backward: [Go back.](#ros2-tutorial-22-services--clients)

## 1. Writing a Service Node

:rotating_light: As a prerquisite, make sure you have created and compiled the [tutorial_service_definition](https://github.com/Woolfrey/tutorial_service_definition) package. :rotating_light:

i) Navigate to your ROS2 workspace source folder and create the new package:
```
cd ~/<workspace_director>/src
ros2 pkg create --dependencies rclcpp tutorial_service_definition -- tutorial_service_client
```
ii) Navigate in to the newly created package and create the `haiku_server.cpp` file:
```
cd tutorial_service_client/src
gedit haiku_server.cpp
```
(You can use whatever IDE you like, besides `gedit`).

iii) Insert the following code in to the file and save:
```
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <tutorial_service_definition/srv/haiku.hpp>                                                // Custom service type

using HaikuService = tutorial_service_definition::srv::Haiku;                                       // Makes code easier to read
  
/**
 * Forward declaration of callback function for the `haiku_service`.
 * @param request A request for the line number of the haiku (1, 2, or 3)
 * @param response A string containing the line of the poem.
 */
void get_line(const std::shared_ptr<HaikuService::Request>   request,
                    std::shared_ptr<HaikuService::Response> response)                                        
{
     // For reference, Haiku.srv:
     //  # Request
     //  int64 line_number
     //  ---
     //  # Response
     //  string line
     
     switch(request->line_number)
     {    
          case 1:
          {
               response->line = "Worker bees can leave.";
               break;
          }
          case 2:
          {
               response->line = "Even drones can fly away.";
               break;
          }
          case 3:
          {
               response->line = "The Queen is their slave.";
               break;
          }
          default:
          {
               response->line = "FLAGRANT REQUEST ERROR: Expected request of 1, 2, or 3 but yours was "
                             + std::to_string(request->line_number);
               break;
          }
     }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
     rclcpp::init(argc,argv);                                                                       // Start up ROS2
     
     std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("haiku_server");                // Needs to be a shared_ptr so we can spin() it later
    
     rclcpp::Service<HaikuService>::SharedPtr service =
     node->create_service<HaikuService>("haiku_service", &get_line);                                // Create service from this node, attach callback

     RCLCPP_INFO(node->get_logger(), "Ready to read you an haiku.");                                // Inform the user
     
     rclcpp::spin(node);                                                                            // Run this node indefinitely
     
     rclcpp::shutdown();                                                                            // Shut down ROS2

     return 0;                                                                                      // No issues with main
}
```
iv) Go back and add the following to the `CMakeLists.txt` file before the `ament_package()` line:
```
add_executable(haiku_server src/haiku_server.cpp)
ament_target_dependencies(haiku_server
                          "rclcpp"
                          "tutorial_service_definition")

# This is needed so ROS can find the package
install(TARGETS
        haiku_server
        DESTINATION lib/${PROJECT_NAME})
```
v) Navigate back to the root of your ROS2 workspace and compile the package:
```
cd ~/<workspace_director>
colcon build --packages-select tutorial_service_definition
```
vi) Source the local setup so that ROS2 can find the newly created package:
```
source ./install.setup.bash
```
vii) You should now be able to launch the service node:
```
ros2 run tutorial_service_client haiku_server
```

:arrow_backward: [Go back.](#ros2-tutorial-22-services--clients)

### :mag: The Code Explained

This substitution is just to make the code neater and more readable:
```
using HaikuService = tutorial_service_definition::srv::Haiku;
```
This is a forward declaration of the callback function for the service:
```
void get_line(const std::shared_ptr<HaikuService::Request>   request,
                    std::shared_ptr<HaikuService::Response>  response)
{
     ...
}
```
It takes the request and response definitions in the `Haiku.srv` file as arguments. ROS2 requires that we specify these arguments as `std::share_ptr<>`.

This line of code starts up ROS2 (if it's not already running):
```
rclcpp::init(argc,argv);
```
Here we create the node:
```
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("haiku_server");
```
It must be a shared pointer so we can call `spin()` on it later.

In this line we:
- Create a service from the previously declared node,
- Advertise the service with the name "haiku_service", and
- Attach the previously declared function `get_line()` as its specific callback.
```
rclcpp::Service<HaikuService>::SharedPtr service = node->create_service<HaikuService>("haiku_service", &get_line);
```
In this line, the node will continuously run and execute its callback whenever requested:
```
rclcpp::spin(node);
```
:arrow_backward: [Go back.](#ros2-tutorial-22-services--clients)

## 2. Writing a Client Node

i) In the direction `tutorial_service_client/src` create a file named `haiku_client.cpp` and add the following code:
```
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <tutorial_service_definition/srv/haiku.hpp>                                                // Custom service type

using HaikuService = tutorial_service_definition::srv::Haiku;                                       // Makes code easier to read

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
     rclcpp::init(argc,argv);                                                                       // Start up ROS2, if not already running
     
     // Minimum for argc is 1 but I don't know why ¯\_(ツ)_/¯
     if(argc != 2)
     {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Incorrect number of arguments. Usage is: `haiku_client X` where X = {1,2,3}.");
          
          return 1;                                                                                 // Exit main(), flag error
     }
     
     std::string temp = argv[1];                                                                    // Convert char to string
             
     std::cout << temp << std::endl;
 
     HaikuService::Request::SharedPtr request = std::make_shared<HaikuService::Request>();          // We need this absolutely ridiculous fucking declaration because ROS2 forces us to use std::shared_pointers everywhere
     
     request->line_number = std::stoi(temp);                                                        // Convert string to int
     
     // Check argument is valid
     if( request->line_number != 1
     and request->line_number != 2
     and request->line_number != 3)
     {
          std::string message = "Expected argument to be 1, 2, or 3 but yours was "
                              + std::to_string(request->line_number) + ".";
                              
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), message.c_str());
          
          return 1;                                                                                 // Exit main(), flag error
     }
       
     rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("haiku_client");                      // Create node with given name
     
     rclcpp::Client<HaikuService>::SharedPtr client = node->create_client<HaikuService>("haiku_service"); // Create client with same name as server
         
     // Loop indefinitely while waiting for service. Exit if cancelled.
     while(not client->wait_for_service(std::chrono::seconds(2)))
     {
          if(not rclcpp::ok())
          {
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Client interrupted while waiting for service. Exiting...");
               
               return 0;                                                                            // Exit main()
          }
     
          RCLCPP_INFO(node->get_logger(), "Waited for service...");                                  // Inform user
     }
 
     auto response = client->async_send_request(request);                                           // WHAT IS THE DATA TYPE?! (ノಠ益ಠ)ノ彡┻━┻

     // Wait for result from server
     if(rclcpp::spin_until_future_complete(node,response) == rclcpp::FutureReturnCode::SUCCESS)
     {
          RCLCPP_INFO(node->get_logger(), response.get()->line.c_str());
     }
     else
     {
          RCLCPP_ERROR(node->get_logger(), "Service request failed.");
     }
     
     rclcpp::shutdown();                                                                            // Shut down ROS2
     
     return 0;                                                                                      // No problems with main
}
```
ii) Modify the `CMakeLists.txt` file in the root of `tutorial_service_client` to compile the new source file:
```
add_executable(haiku_client src/haiku_client.cpp)
ament_target_dependencies(haiku_client
                          "rclcpp"
                          "tutorial_service_definition")
```
iii) Don't forget to modify the install targets so that it can be found by ROS:
```
# This is needed so ROS can find the package
install(TARGETS
        haiku_server
        haiku_client
        DESTINATION lib/${PROJECT_NAME})
```
iv) Now navigate back to your root ROS workspace directory and build:
```
colcon build --packages-select tutorial_service_client
```
v) Be sure to re-source so that it can be found by ROS:
```
source ./install/setup.bash
```
vi) Assuming that the `haiku_server` node is already running, we can run the client node:
```
ros2 run tutorial_service_client haiku_server 1
```
Launching the node successfully requires an integer argument between 1, 2 and 3. You should see something akin to:

<img src="https://github.com/Woolfrey/tutorial_service_client/assets/62581255/bed80cf9-c4d0-4730-be34-1c9d3befd7f7" alt="image" width="900" height="auto">


:arrow_backward: [Go back.](#ros2-tutorial-22-services--clients)

### :mag: The Code Explained

This line starts up ROS2 (if not already running):
```
rclcpp::init(argc,argv);
```
This insanely convoluted line creates the request data type specified by `Haiku.srv` in the `tutorial_service_definition` package:
```
HaikuService::Request::SharedPtr request = std::make_shared<HaikuService::Request>();
```
then in these two lines we convert our input argument from a `char` to a `string` to an `int`:
```
std::string temp = argv[1];
request->line_number = std::stoi(temp);
```
There's probably a smarter way to do this, but I've spent too much time on this already :man_shrugging:.

This just ensures that the program cannot proceed unless the user has input the right value, since a Haiku has only 3 lines:
```
if( request->line_number != 1 and request->line_number != 2 and request->line_number != 3)
{
     ...
}
```
In these lines we create the node named "haiku_client" and generate a client object from it:
```
rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("haiku_client");
rclcpp::Client<HaikuService>::SharedPtr client = node->create_client<HaikuService>("haiku_service");
```

This code will loop indefinitely while it waits for the server to appear. It can be cancelled manually:
```
while(not client->wait_for_service(std::chrono::seconds(2)))
{
     ...
}
```

This line sends the request to the server:
```
auto response = client->async_send_request(request);
```
(I _really_ don't like the use of `auto` here, but ROS2 is extremely convoluted...)

Finally, this code determines if the response has been successfully processed and retrieved:
```
if(rclcpp::spin_until_future_complete(node,response) == rclcpp::FutureReturnCode::SUCCESS)
{
     ...
}
```

:arrow_backward: [Go back.](#ros2-tutorial-22-services--clients)
