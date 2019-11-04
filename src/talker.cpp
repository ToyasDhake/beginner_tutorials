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
*/
#include <beginner_tutorials/serviceMessage.h>

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

bool callback(beginner_tutorials :: serviceMessageRequest &request,
    beginner_tutorials :: serviceMessageResponse &response) {
  ROS_INFO("callback activated");
  std::string in_name(request.operation);
  response.onList = false;
  // Perform calculations
  if (in_name.compare("ADD") == 0) {
    ROS_DEBUG_STREAM("Perfoming addition.");
    response.answer = request.num1+request.num2;
    response.onList = true;
  }
  else if (in_name.compare("SUB") == 0) {
    ROS_DEBUG_STREAM("Perfoming addition.");
    response.answer = request.num1-request.num2;
    response.onList = true;
  }
  else if (in_name.compare("MUL") == 0) {
    ROS_DEBUG_STREAM("Perfoming addition.");
    response.answer = request.num1*request.num2;
    response.onList = true;
  }
  else if (in_name.compare("DIV") == 0) {
    ROS_DEBUG_STREAM("Perfoming addition.");
    response.answer = request.num1/request.num2;
    response.onList = true;
  }
  return true;
}


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
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("Something is worng ROS node not initialized");
  }
  ROS_DEBUG_STREAM("Ready to compute.");
  ros::ServiceServer service = n.advertiseService("calculator", callback);
  int rate = 10;
  if (argv[1] == "fast") {
    rate = 30;
  }
  ros::Rate loop_rate(rate);
  loop_rate.sleep();
  ros::spin();


  return 0;
}
