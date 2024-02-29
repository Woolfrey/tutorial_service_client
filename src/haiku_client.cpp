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
             
     HaikuService::Request::SharedPtr request = std::make_shared<HaikuService::Request>();          // We need this absolutely ridiculous fucking declaration because ROS2 forces us to use std::shared_pointers everywhere
     
     std::string temp = argv[1];                                                                    // Convert char to string
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
