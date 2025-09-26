#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <ros/console.h>

#include <stdio.h>
#include <math.h>
#include <stdint.h>

// Just for niceness
typedef int64_t i64;
typedef int32_t i32;
typedef int16_t i16;
typedef int8_t  i8;

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef int64_t b64;
typedef int32_t b32;
typedef int16_t b16;
typedef int8_t  b8;

typedef double f64;
typedef float  f32;

#define EVENT_QUEUE_COUNT 10

struct Bot
{
  ros::NodeHandle handle;
  ros::Publisher  velocity_publisher;
  ros::Subscriber bumper_subscriber;
  ros::Subscriber scan_subscriber;
  ros::Subscriber odom_subscriber;

  bool should_halt = false;

  i32  distance_since_turned_feet = 0.0;
  f64  prev_x = 0.0;
  f64  prev_y = 0.0;

  Bot()
  {
    velocity_publisher = handle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",
                                                                EVENT_QUEUE_COUNT);

    bumper_subscriber = handle.subscribe("/mobile_base/events/bumper", EVENT_QUEUE_COUNT,
                                          &Bot::bumper_callback, this);

    odom_subscriber = handle.subscribe("/odom", EVENT_QUEUE_COUNT,
                                       &Bot::odom_callback, this);

  }

  void spin()
  {
    ros::Rate rate(10); // ? good rate?

    while (ros::ok())
    {
      // Callback stuff
      ros::spinOnce();

      geometry_msgs::Twist cmd;

      if (should_halt)
      {
        // STOP!
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
      }
      // Obstacles here somehow??
      else if (distance_since_turned_feet > 1.0)
      {
        // Turn in random 15 degrees
      }
      else
      {
        cmd.linear.x  = 0.4;
      }

      velocity_publisher.publish(cmd);

      rate.sleep();
    }
  }

  void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
      should_halt = true;
    }
    else
    {
      should_halt = false;
    }
  }

  void scan_callback()
  {

  }


  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    f64 x = msg->pose.pose.position.x;
    f64 y = msg->pose.pose.position.y;
    f64 dx = msg->pose.pose.position.x - prev_x;
    f64 dy = msg->pose.pose.position.y - prev_y;

    // Pythagorean!
    f64 dist_meters = sqrt(dx * dx - dy * dy);

    #define FEET_PER_METER 3.28084
    distance_since_turned_feet += (dist_meters * FEET_PER_METER);

    prev_x = x;
    prev_y = y;
  }
};

int main(int arg_count, char **args)
{
  ros::init(arg_count, args, "project1_turtlebot");

  Bot bot{};
  bot.spin();
}
