/*
* Copyright [MIT] 2019 Toyas Dhake
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of
* this software and associated documentation files (the "Software"), to deal in
* the Software without restriction, including without limitation the rights to
* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
* the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
* FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
* IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* @file listener.cpp
* @author Toyas Dhake
* @date 04 Nov 2019
* @copyright 2019 Toyas Dhake
* @brief Client implementation to request service to perform simple arithmatic 
* operations.
*/
#include <beginner_tutorials/serviceMessage.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "listener");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("Something is worng ROS node not initialized");
    }
    // Create Client
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::
    serviceMessage>("calculator");
    beginner_tutorials :: serviceMessage srv;
    std::string operation;

    float num1, num2;
    bool divideByZeroCheck = true;
    while (ros::ok()) {
        std::cout << std::endl;
        std::cout << "Enter operation to perform (enter <exit> to exit):- "
                                                                << std::endl;
        ROS_INFO_STREAM("Following operations can be done- ADD, SUB, MUL, DIV");
        // Enter the operation to perform
        std::cin >> operation;

        if (operation.compare("exit") == 0) {
            ROS_WARN_STREAM("Exiting...");
            return 0;
        }
        // Enter first number
        std::cout << "Enter first number:- ";
        std::cin >> num1;
        // Enter second number
        std::cout << "Enter first number:- ";
        std::cin >> num2;

        // Check for divide by zero
        if (operation.compare("DIV") == 0) {
            if (num2 == 0) {
                ROS_ERROR_STREAM("You trying to divide by zero.");
                divideByZeroCheck = false;
            }
        }
        if (divideByZeroCheck) {
            // Assign values ot request variables
            srv.request.operation = operation;
            srv.request.num1 = num1;
            srv.request.num2 = num2;
            // Call the service
            if (client.call(srv)) {
                // Print result
                if (srv.response.onList) {
                    std::cout << "Answer is:- " << srv.response.answer
                                                                << std::endl;
                } else {
                    std::cout << srv.request.operation <<
                                        " is not valid operation." << std::endl;
                }

            } else {
                // Service call failed
                ROS_ERROR_STREAM("Failed to call service.");
                return 1;
            }
        }
    }
    return 0;
}
