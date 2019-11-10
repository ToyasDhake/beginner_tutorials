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
* @brief Broadcasts tf Fram consisting of non zero translation in x, y and z 
*        direction and rotation in quaternion fromat.
*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/**
* @brief Broadcast TF frame with non zero value of translation in x, y and z 
*        direction and rotation i nfrom of quaternion.
* @params double x X coordiante
* @params double y Y coordiante
* @params double z Z coordiante
* @params double yaw Yaw rotation
* @params double pitch Pitch rotation
* @params double roll Roll rotation
* @params std::string name Name of TF frame.
* @return void
*/
void broadcastTF(double x, double y, double z, double yaw, double pitch,
                                               double roll, std::string name) {
    static tf::TransformBroadcaster br;
    // Create TF frame and set values
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform.setRotation(q);
    // Broadcast frame
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                         "world", name));
}

int main(int argc, char** argv) {
    std::string name;
    ros::init(argc, argv, "my_tf_broadcaster");
    if (argc != 2) {
        ROS_ERROR("need a name as argument");
        return -1;
    }
    name = argv[1];

    // Set limit to refresh rate
    ros::NodeHandle node;
    ros::Rate rate(10);

    while (ros::ok()) {
        // Set and broadcast
        broadcastTF(1, 2, 3, 180, -90, 90, name);
        
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
