#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>
#include <memory>
#include <random>

// helpers
static inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline double containTheAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

static inline double toRadians(double d)
{
  return d * M_PI / 180.0;
}

static double getYaw(const geometry_msgs::Quaternion& q)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(q, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

// shared state
struct Context
{
  ros::NodeHandle handle_ {"~"};

  //// various properties that can be adjusted to fine tune the robot
  // forward driving speed
  double forward_speed_ {0.22};

  // turning speed
  double turn_speed_ {0.9};

  // strongness to steer away from obstacles
  double avoid_strength_ {1.5};

  // are left/right detections equal (do we need to escape)
  double avoid_range_ {0.1};

  // distance before we turn/escape
  double front_stopping_dist_ {2};

  // forward fov
  double front_fov_ {20};

  // side fov
  double side_fov_ {20};

  // the range of random turning
  double random_turn_deg_ {15};

  // the random of escape turning
  double random_escape_turn_deg_ {30};

  // when we can mark a turn as completed (+/- this number)
  double target_angle_error_ {12.0};

  // minimum avoid turn speed (fixes stuck issues)
  double min_turn_speed_ {0.3};

  //// state stuff
  bool bumped_ {false};
  bool have_scan_ {false};
  bool have_odom_ {false};
  bool teleop_block_move_ {false};
  sensor_msgs::LaserScan scan_;
  nav_msgs::Odometry odom_;
  ros::Time last_teleop_time_ {0};
  geometry_msgs::Twist current_command_;

  bool escape_active_ {false};
  bool random_active_ {false};
  double escape_target_yaw_ {0.0};
  double random_target_yaw_ {0.0};

  double last_random_x_ {0.0};
  double last_random_y_ {0.0};
  double rand_turn_dist_ {0.3048};

  std::mt19937 rng_;

  Context() : rng_(static_cast<unsigned>(ros::Time::now().toNSec())) {}

  double yaw()
  {
    return have_odom_ ? getYaw(odom_.pose.pose.orientation) : 0.0;
  }

  // finds the closest obstacle within the ang_lo to ang_hi given by the scan data
  // passes to out_min
  bool checkRange(double ang_lo, double ang_hi, double& out_min)
  {
    // return false immediately if we don't have a scan yet
    if (!have_scan_)
    {
      return false;
    }

    // if no scan available, nothing to check
    const auto& scan = scan_;
    int num_ranges = static_cast<int>(scan.ranges.size());
    if (num_ranges == 0)
    {
      return false;
    }

    // function to check if a given angle lies within ang_lo to ang_hi
    auto inAngleSlice = [&](double angle) -> bool
    {
      double mid_angle = containTheAngle((ang_lo + ang_hi) * 0.5);
      double width = std::fabs(containTheAngle(ang_hi - ang_lo));
      double diff = std::fabs(containTheAngle(angle - mid_angle));
      return diff <= width * 0.5 + 1e-6;
    };

    // this is the first time I learned about numeric_limits, a cool little "utility" structure
    double min_range = std::numeric_limits<double>::infinity();
    for (int i = 0; i < num_ranges; ++i)
    {
      double angle = scan.angle_min + i * scan.angle_increment;
      if (!std::isfinite(scan.ranges[i]))
      {
        continue;
      }

      // if it falls within the slice, if so check for new min
      if (inAngleSlice(angle))
      {
        min_range = std::min(min_range, static_cast<double>(scan.ranges[i]));
      }
    }

    // some weird stuff was happening with really small numbers
    if (!std::isfinite(min_range))
    {
      return false;
    }

    out_min = min_range;
    return true;
  }

  // checks the front
  bool frontCheck(double half_fov, double& out_min)
  {
    return checkRange(-toRadians(half_fov), toRadians(half_fov), out_min);
  }

  // checks the left half of the sensor
  bool leftCheck(double half_fov, double& out_min)
  {
    return checkRange(toRadians(5.0), toRadians(half_fov), out_min);
  }

  // checks the right half of the sensor
  bool rightCheck(double half_fov, double& out_min)
  {
    return checkRange(-toRadians(half_fov), -toRadians(5.0), out_min);
  }
};

// behavior base
class Behavior
{
public:
  explicit Behavior(Context& ctx) : ctx_(ctx) {}
  virtual ~Behavior() {}
  virtual bool run() = 0;

protected:
  Context& ctx_;
};

// behaviors
class BumperBehavior : public Behavior
{
public:
  explicit BumperBehavior(Context& ctx) : Behavior(ctx) {}

  bool run() override
  {
    if (ctx_.bumped_)
    {
      ctx_.current_command_ = geometry_msgs::Twist();
      if (!teleop_killed_)
      {
        // todo: this is probably a bad way of doing it, but it does work (I think ROS launch files can autokill based on dynamic params)
        int ret_val = system("rosnode kill /turtlebot_teleop_keyboard");
        ROS_WARN("Bumper Triggered :( | killing teleop node = %d", ret_val);
        teleop_killed_ = true;
      }
      return true;
    }
    return false;
  }

private:
  bool teleop_killed_ {false};
};

class EscapeBehavior : public Behavior
{
public:
  explicit EscapeBehavior(Context& ctx) : Behavior(ctx) {}

  bool run() override
  {
    if (!ctx_.have_scan_)
    {
      return false;
    }

    double lmin, rmin;
    bool lh = ctx_.leftCheck(ctx_.side_fov_, lmin);
    bool rh = ctx_.rightCheck(ctx_.side_fov_, rmin);

    bool left_close = lh && lmin < ctx_.front_stopping_dist_ * 0.3048;
    bool right_close = rh && rmin < ctx_.front_stopping_dist_ * 0.3048;
    bool symmetric = left_close && right_close && std::fabs(lmin - rmin) <= ctx_.avoid_range_;

    double fmin;
    bool fh = ctx_.frontCheck(ctx_.front_fov_, fmin);
    symmetric = symmetric || (fh && fmin < ctx_.front_stopping_dist_ * 0.3048 && (!left_close && !right_close));

    if (!ctx_.escape_active_ && symmetric)
    {
      std::uniform_real_distribution<double> jitter(-toRadians(ctx_.random_escape_turn_deg_),
                                                    toRadians(ctx_.random_escape_turn_deg_));
      ctx_.escape_target_yaw_ = containTheAngle(ctx_.yaw() + M_PI + jitter(ctx_.rng_));
      ctx_.escape_active_ = true;
    }

    if (ctx_.escape_active_)
    {
      double err = containTheAngle(ctx_.escape_target_yaw_ - ctx_.yaw());
      double sgn = (err >= 0.0) ? 1.0 : -1.0;
      geometry_msgs::Twist t;
      t.angular.z = sgn * ctx_.turn_speed_;
      ctx_.current_command_ = t;
      if (std::fabs(err) < toRadians(ctx_.target_angle_error_))
      {
        ctx_.escape_active_ = false;
      }
      return true;
    }
    return false;
  }
};

class AvoidBehavior : public Behavior
{
public:
  explicit AvoidBehavior(Context& ctx) : Behavior(ctx) {}

  bool run() override
  {
    if (!ctx_.have_scan_)
    {
      return false;
    }

    // basically calculate left and right side objects to determine where we go
    double front_m = ctx_.front_stopping_dist_ * 0.3048;
    double lmin, rmin, fmin;
    bool lh = ctx_.leftCheck(ctx_.side_fov_, lmin);
    bool rh = ctx_.rightCheck(ctx_.side_fov_, rmin);
    bool fh = ctx_.frontCheck(ctx_.front_fov_, fmin);
    bool left_close = lh && lmin < front_m;
    bool right_close = rh && rmin < front_m;
    bool front_close = fh && fmin < front_m;

    // no objects no dice
    if (!left_close && !right_close && !front_close)
    {
      return false;
    }

    if (left_close && right_close && std::fabs(lmin - rmin) <= ctx_.avoid_range_)
    {
      return false;
    }

    // what way do we need to turn based on sensor data
    double turn = 0.0;
    if (left_close && (!right_close || lmin < rmin))
    {
      turn = -ctx_.avoid_strength_ * (front_m - lmin);
    }
    else if (right_close)
    {
      turn = ctx_.avoid_strength_ * (front_m - rmin);
    }
    else if (front_close)
    {
      // slow down a bit when we get close to something directlyt in front
      turn = 0.8 * ctx_.turn_speed_;
    }

    double ang = clamp(turn, -ctx_.turn_speed_, ctx_.turn_speed_);
    if (std::fabs(ang) < ctx_.min_turn_speed_)
    {
      ang = (ang >= 0.0 ? ctx_.min_turn_speed_ : -ctx_.min_turn_speed_);
    }
    
    geometry_msgs::Twist t;
    t.angular.z = ang;
    t.linear.x = 0.0;
    ctx_.current_command_ = t;
    return true;
  }
};

class RandomTurnBehavior : public Behavior
{
public:
  explicit RandomTurnBehavior(Context& ctx) : Behavior(ctx) {}

  bool run() override
  {
    if (!ctx_.have_odom_)
    {
      return false;
    }

    // todo: this is broken I think I'm just slow
    if ((ros::Time::now() - ctx_.last_teleop_time_).toSec() <= 2 || ctx_.teleop_block_move_) 
    {
      return false;
    }

    double x = ctx_.odom_.pose.pose.position.x;
    double y = ctx_.odom_.pose.pose.position.y;

    // todo: move outside of function (c++ probably optimizes this out anyway)
    auto dist = [&](double x0, double y0, double x1, double y1)
    {
      return std::hypot(x1 - x0, y1 - y0);
    };

    if (!ctx_.random_active_)
    {
      double d = dist(ctx_.last_random_x_, ctx_.last_random_y_, x, y);
      if (d >= ctx_.rand_turn_dist_)
      {
        std::uniform_real_distribution<double> dtheta(-toRadians(ctx_.random_turn_deg_),
                                                      toRadians(ctx_.random_turn_deg_));
        ctx_.random_target_yaw_ = containTheAngle(ctx_.yaw() + dtheta(ctx_.rng_));
        ctx_.random_active_ = true;
        ctx_.last_random_x_ = x;
        ctx_.last_random_y_ = y;
      }
      else
      {
        return false;
      }
    }

    double err = containTheAngle(ctx_.random_target_yaw_ - ctx_.yaw());
    geometry_msgs::Twist t;
    t.angular.z = ((err >= 0.0) ? 1.0 : -1.0) * (0.6 * ctx_.turn_speed_);
    ctx_.current_command_ = t;

    if (std::fabs(err) < toRadians(ctx_.target_angle_error_))
    {
      ctx_.random_active_ = false;
    }
    return true;
  }
};

class ForwardBehavior : public Behavior
{
public:
  explicit ForwardBehavior(Context& ctx) : Behavior(ctx) {}

  bool run() override
  {
    // todo: this is broken I think I'm just slow
    geometry_msgs::Twist t;
    if ((ros::Time::now() - ctx_.last_teleop_time_).toSec() <= 2 || ctx_.teleop_block_move_) 
    {
      return false;
    }

    t.linear.x = ctx_.forward_speed_;
    ctx_.current_command_ = t;
    return true;
  }
};

// bot
class Bot
{
public:
  Bot() : ctx_()
  {
    pub_cmd_ = ctx_.handle_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    sub_bumper_ = ctx_.handle_.subscribe("/mobile_base/sensors/bumper_pointcloud", 1, &Bot::bumperCallback, this);
    sub_scan_ = ctx_.handle_.subscribe("/scan", 1, &Bot::scanCallback, this);
    sub_odom_ = ctx_.handle_.subscribe("/odom", 1, &Bot::odomCallback, this);
    sub_teleop_ = ctx_.handle_.subscribe("/cmd_vel_mux/input/teleop", 1, &Bot::teleopCallback, this);

    behaviors_.emplace_back(new BumperBehavior(ctx_));
    behaviors_.emplace_back(new EscapeBehavior(ctx_));
    behaviors_.emplace_back(new AvoidBehavior(ctx_));
    behaviors_.emplace_back(new RandomTurnBehavior(ctx_));
    behaviors_.emplace_back(new ForwardBehavior(ctx_));
  }

  void spin()
  {
    ros::Rate rate(20.0);
    while (ros::ok())
    {
      ros::spinOnce();
      ctx_.current_command_ = geometry_msgs::Twist();

      // loop through behaviors and break after the first one edits the command (priority list)
      for (auto& b : behaviors_)
      {
        if (b->run())
        {
          break;
        }
      }

      pub_cmd_.publish(ctx_.current_command_);
      rate.sleep();
    }
  }

private:
  Context ctx_;
  ros::Publisher pub_cmd_;
  ros::Subscriber sub_bumper_, sub_scan_, sub_odom_, sub_teleop_;
  std::vector<std::unique_ptr<Behavior>> behaviors_;

  void bumperCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    ctx_.bumped_ = (msg->width > 0 && msg->height > 0);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    ctx_.scan_ = *msg;
    ctx_.have_scan_ = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    ctx_.odom_ = *msg;
    ctx_.have_odom_ = true;
    static bool init = false;
    if (!init)
    {
      ctx_.last_random_x_ = ctx_.odom_.pose.pose.position.x;
      ctx_.last_random_y_ = ctx_.odom_.pose.pose.position.y;
      init = true;
    }
  }

  // im unsure if this even works
  void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {    
    if (std::fabs(msg->linear.y) > 1e-3 ||
    std::fabs(msg->linear.z) > 1e-3 ||
    std::fabs(msg->angular.x) > 1e-3 ||
    std::fabs(msg->angular.y) > 1e-3) 
    {
      ctx_.last_teleop_time_ = ros::Time::now();
      ctx_.teleop_block_move_ = true;
    } else {
      ctx_.teleop_block_move_ = false;
    }
  }
};

// main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "project1_turtlebot");
  Bot bot;
  bot.spin();
  return 0;
}