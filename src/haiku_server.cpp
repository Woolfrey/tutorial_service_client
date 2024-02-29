#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <tutorial_service_definition/srv/haiku.hpp>                                                // Custom service type

using HaikuService = tutorial_service_definition::srv::Haiku;                                       // Makes code easier to read
  
/**
 * Forward declaration of callback function for the `haiku_service`.
 * @param request A request for the line number of the haiku (1, 2, or 3)
 * @param response A string containing the line of the poem.
 */
void get_line(const std::shared_ptr<HaikuService::Request>   request,
                    std::shared_ptr<HaikuService::Response>  response)                                        
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
