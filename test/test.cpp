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
 * @file       test.cpp
 * @author     Adarsh Jagan Sathyamoorthy
 * @copyright  3-clause BSD
 * @brief      rostest program using gtest frame
 * A ROStest using gtest frame work to test services in the talker node
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/giveNewString.h"
#include "std_msgs/String.h"
/**
 * @brief Checks if changePublishedString service exists
 */
TEST(testService, testServiceExistence) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<beginner_tutorials::giveNewString>("changePublishedString");
    EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}
/**
 * @brief  tests if service changePublishedString replaces default string
 *         with user's string
 */
TEST(testService, testStringUpdate) {
    ros::NodeHandle nh;
    auto client =
 nh.serviceClient<beginner_tutorials::giveNewString>("changePublishedString");
    beginner_tutorials::giveNewString::Request request;
    beginner_tutorials::giveNewString::Response response;
    request.inputString = "Test";
    client.call(request, response);
    EXPECT_STREQ("Test", response.outputString.c_str());
}
