#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>
#include <stack>
#include <cassert>
#include <unordered_map>
#include <sstream>
#include <fstream>
#include <memory>

#include <ros/package.h>

struct PointFt {
    double x{0.0}, y{0.0};
};

struct Landmark {
    PointFt location;
    double priority;
};

static double dist(double x1, double y1, double x2, double y2) {
    const double dx = x2 - x1, dy = y2 - y1;
    return std::hypot(dx, dy);
}

static double landmark_weight(const Landmark& from, const Landmark& to)
{
    double distance = dist(from.location.x, from.location.y,
                           to.location.x, to.location.y);

    double factor = 0.2;
    double priority_bias = factor * (from.priority + to.priority);

    double result = distance - priority_bias;
    if (result < 0.0001) result = 0.0001;
    return result;
}

static std::vector<PointFt> two_opt_path(const std::vector<Landmark> &points)
{
    if (points.empty()) {
        return {};
    }

    const size_t points_count = points.size();

    struct MST_Point {
        bool in_mst = false;
        double min_edge =_
