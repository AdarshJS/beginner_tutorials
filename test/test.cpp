#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/giveNewString.h"
#include "std_msgs/String.h"

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
