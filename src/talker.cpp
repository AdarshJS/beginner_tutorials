/**
 * Copyright (c) 2018, Adarsh Jagan Sathyamoorthy
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file       talker.cpp
 * @author     Adarsh Jagan Sathyamoorthy
 * @copyright  3-clause BSD
 * @brief      A ROS publisher
 * A basic ROS publisher that broadcasts a string (my name in this case).
 */
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "talker.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/giveNewString.h"

/**
 * Published string when nothing is modified
 */
DefString s;

/**
 * @brief         Service function for changePublishedString Service
 * @param  req    Request of the service (contains input string to be published)
 * @param  resp   Response of the service (contains modified output string)
 * @return bool
 */
bool giveNewStringCb(beginner_tutorials::giveNewString::Request& req,
beginner_tutorials::giveNewString::Response& resp) {
    ROS_INFO("Requested new string: %s", req.inputString.c_str());
    // Change the default string based on request.
    s.defString = req.inputString;
    resp.outputString = req.inputString;
    ROS_WARN_STREAM("Default string changed.");
    return true;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  // Accepting argument and changing loop Rate
  int loopRate = 10;

  if (argc == 2) {
    loopRate = atoi(argv[1]);
    ROS_DEBUG_STREAM("Changed loop rate is: " << loopRate);
  }
  if (loopRate < 0) {
        ROS_ERROR_STREAM("Entered loop rate is negative!");
        loopRate = 10;
        ROS_WARN_STREAM("Loop rate changed to 10Hz (Default).");
    }
  if (loopRate == 0) {
       ROS_FATAL_STREAM("Loop rate cannot be zero!");
       loopRate = 10;
       ROS_WARN_STREAM("Loop rate changed to 10Hz (Default).");
    }

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
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Advertise the service
  auto newStringServer = n.advertiseService("changePublishedString", giveNewStringCb);

  ros::Rate loop_rate(loopRate);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << s.defString << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    // Broadcasting the transform
    transform.setOrigin(tf::Vector3(3.0, 4.0, 0.0));
    tf::Quaternion q;
    // Rotation about Z set to pi/2
    q.setRPY(0, 0, 1.57);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
