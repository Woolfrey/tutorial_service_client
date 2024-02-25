#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>                                                                  // String message type
#include <tutorial_service_definition/srv/haiku.hpp>


using HaikuService = tutorial_service_definition::srv::Haiku;
 
void getline(const HaikuService::Request &request, HaikuService::Response &response)                                        
{
     // For reference, Haiku.srv:
     //  # Request
     //  int64 line_number
     //  ---
     //  # Response
     //  string line
     
     switch(request.line_number)
     {    
          case 1:
          {
               response.line = "Worker bees can leave.";
               break;
          }
          case 2:
          {
               response.line = "Even drones can fly away.";
               break;
          }
          case 3:
          {
               response.line = "The Queen is their slave.";
               break;
          }
          default:
          {
               response.line = "FLAGRANT REQUEST ERROR: Expected request of 1, 2, or 3 but yours was "
                             + std::to_string(request.line_number);
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
     
     rclcpp::Node node("haiku_server");                                                             // Create node with name `haiku_server`

     rclcpp::Service<HaikuService>::SharedPtr srvc = node.create_service<HaikuService>("haiku_service", &getline);

/*
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
   
     std::cout << "Worker bees can leave.\n";
     std::cout << "Even drones can fly away.\n";
     std::cout << "The Queen is their slave.\n";
     */
     
     rclcpp::shutdown();                                                                            // Shut down ROS2
     
     return 0;                                                                                      // No issues with main
}
