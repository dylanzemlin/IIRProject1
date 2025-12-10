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
        double min_edge = std::numeric_limits<double>::infinity();
        size_t parent_index = 0;
        std::vector<size_t> neighbor_indices;
        bool visited = false;
    };

    std::vector<MST_Point> mst(points_count);
    mst[0].in_mst = true;

    for (size_t i = 1; i < points_count; i++)
        mst[i].min_edge = landmark_weight(points[0], points[i]);

    for (size_t edge = 0; edge < points_count - 1; edge++)
    {
        double min_edge = std::numeric_limits<double>::infinity();
        size_t add_idx = static_cast<size_t>(-1);

        for (size_t idx = 0; idx < points_count; idx++)
        {
            if (!mst[idx].in_mst && mst[idx].min_edge < min_edge)
            {
                min_edge = mst[idx].min_edge;
                add_idx = idx;
            }
        }

        if (add_idx == static_cast<size_t>(-1)) {
            break;
        }

        mst[add_idx].in_mst = true;
        size_t parent = mst[add_idx].parent_index;

        mst[parent].neighbor_indices.push_back(add_idx);
        mst[add_idx].neighbor_indices.push_back(parent);

        for (size_t idx = 0; idx < points_count; idx++)
        {
            if (!mst[idx].in_mst)
            {
                double d = landmark_weight(points[add_idx], points[idx]);
                if (d < mst[idx].min_edge)
                {
                    mst[idx].min_edge = d;
                    mst[idx].parent_index = add_idx;
                }
            }
        }
    }

    std::vector<size_t> tour;
    tour.reserve(points_count + 1);
    std::stack<size_t> stack;
    stack.push(0);

    while (!stack.empty())
    {
        size_t current = stack.top();
        stack.pop();

        if (mst[current].visited)
            continue;

        mst[current].visited = true;
        tour.push_back(current);

        for (size_t n = mst[current].neighbor_indices.size(); n-- > 0;)
        {
            size_t idx = mst[current].neighbor_indices[n];
            if (!mst[idx].visited)
                stack.push(idx);
        }
    }

    // Close the tour by returning to the first index
    tour.push_back(0);

    // 2-opt improvement
    bool improved = true;
    while (improved)
    {
        improved = false;
        for (size_t i = 0; i + 3 < tour.size(); i++)
        {
            for (size_t j = i + 2; j + 1 < tour.size(); j++)
            {
                Landmark a = points[tour[i]];
                Landmark b = points[tour[i+1]];
                Landmark c = points[tour[j]];
                Landmark d = points[tour[j+1]];

                double before = landmark_weight(a, b) + landmark_weight(c, d);
                double after  = landmark_weight(a, c) + landmark_weight(b, d);

                if (after < before)
                {
                    std::reverse(tour.begin() + i + 1, tour.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }

    std::vector<PointFt> out;
    out.reserve(tour.size());
    for (size_t idx : tour)
        out.push_back(points[idx].location);

    return out;
}

static std::unordered_map<std::string, Landmark> load_waypoint_file(const std::string& path)
{
    std::unordered_map<std::string, Landmark> table;

    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open waypoint file: %s", path.c_str());
        return table;
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty()) continue;

        // Expected: Name: x, y
        std::stringstream ss(line);
        std::string name, coords;

        if (!std::getline(ss, name, ':')) continue;
        if (!std::getline(ss, coords))     continue;

        double x, y;
        if (sscanf(coords.c_str(), " %lf , %lf", &x, &y) == 2)
        {
            Landmark lm;
            lm.location = {x, y};
            lm.priority = 0.0;
            table[name] = lm;
        }
    }

    ROS_INFO("Loaded %lu waypoints from file.", static_cast<unsigned long>(table.size()));
    return table;
}

struct Context {
    ros::NodeHandle nh;
    ros::Subscriber tour_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber log_sub;

    ros::Publisher plan_pub;
    ros::Publisher cmd_vel_pub;

    std::unordered_map<std::string, Landmark> landmark_table;

    std::vector<PointFt> plan;
    size_t current_plan_index = 0;

    double current_speed = 0.0;
    std::string last_log;

    // Pose tracking
    double current_x = 0.0;
    double current_y = 0.0;
    bool has_current_pose = false;

    // Tour timing & start pose
    PointFt start_pose;
    bool has_start_pose = false;
    bool tour_active = false;
    double tour_start_time = 0.0;
    double tour_time_budget = -1.0; // seconds, <0 means no budget

    // Pause flag when time expires
    bool paused_by_timeout = false;
};

class Behavior {
public:
    explicit Behavior(Context& c) : ctx(c) {}
    virtual ~Behavior() = default;
    virtual bool run() = 0;
protected:
    Context& ctx;
};

class MoveBaseBehavior : public Behavior {
public:
    using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

    explicit MoveBaseBehavior(Context& c)
        : Behavior(c), ac("move_base", true)
    {
        ROS_INFO("Waiting for move_base...");
        ac.waitForServer();
        ROS_INFO("Connected to move_base.");
    }

    bool run() override
    {
        // If paused due to timeout, do nothing
        if (ctx.paused_by_timeout) {
            return false;
        }

        if (ctx.current_plan_index >= ctx.plan.size())
            return false;

        const PointFt& p = ctx.plan[ctx.current_plan_index];

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = p.x;
        goal.target_pose.pose.position.y = p.y;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal %zu: (%.2f, %.2f)",
                 ctx.current_plan_index, p.x, p.y);

        ac.sendGoal(goal);
        ac.waitForResult();

        auto st = ac.getState();
        if (st == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Reached waypoint.");
        else
            ROS_WARN("Failed to reach waypoint: %s", st.toString().c_str());

        ctx.current_plan_index++;
        return true;
    }

private:
    MoveBaseClient ac;
};

class Bot {
public:
    Bot()
    {
        std::string path = ros::package::getPath("major_project") + "/waypoints.tour";
        ctx.landmark_table = load_waypoint_file(path);

        ctx.tour_sub = ctx.nh.subscribe("/tour_start", 1, &Bot::tourCallback, this);
        ctx.odom_sub = ctx.nh.subscribe("/odom", 1, &Bot::odomCallback, this);
        ctx.log_sub  = ctx.nh.subscribe("/rosout_agg", 100, &Bot::logCallback, this);

        ctx.plan_pub = ctx.nh.advertise<nav_msgs::Path>("/tour_plan_path", 1, true);
        ctx.cmd_vel_pub = ctx.nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

        behaviors.push_back(std::make_unique<MoveBaseBehavior>(ctx));
    }

    void tourCallback(const std_msgs::String::ConstPtr& msg)
    {
        const std::string data_raw = msg->data;

        // Handle control actions first
        if (data_raw == "ACTION=STOP_NOW") {
            ROS_INFO("Tour: time expired, stopping robot until user decides.");
            stopRobot();
            ctx.paused_by_timeout = true;
            return;
        }

        if (data_raw == "ACTION=CONTINUE") {
            ROS_INFO("Tour: user chose to continue. Disabling time budget and resuming.");
            ctx.tour_time_budget = -1.0;
            ctx.paused_by_timeout = false;
            ctx.tour_active = false;
            return;
        }

        if (data_raw == "ACTION=RETURN") {
            ROS_INFO("Tour: user requested return to start.");
            stopRobot();
            ctx.paused_by_timeout = false;

            if (!ctx.has_start_pose) {
                ROS_WARN("Return requested but start pose is unknown.");
                return;
            }

            ctx.plan.clear();
            ctx.plan.push_back(ctx.start_pose);
            ctx.current_plan_index = 0;
            ctx.tour_active = false;

            publishPlan();
            ROS_INFO("Published return-to-start plan.");
            return;
        }

        // Otherwise parse a new tour request:
        ROS_INFO("UI Requested Tour: %s", data_raw.c_str());

        std::string data = data_raw;
        double time_budget = -1.0;

        // Parse optional TIME=xxx; prefix
        size_t semi = data.find(';');
        if (semi != std::string::npos) {
            std::string time_part = data.substr(0, semi);
            if (time_part.rfind("TIME=", 0) == 0) {
                time_budget = atof(time_part.substr(5).c_str());
                if (time_budget <= 0.0) {
                    time_budget = -1.0;
                }
                ROS_INFO("Received time budget: %.2f seconds", time_budget);
            }
            data = data.substr(semi + 1);
        }

        std::stringstream ss(data);
        std::string token;
        std::vector<Landmark> selected;

        while (std::getline(ss, token, ','))
        {
            // Remove spaces
            token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
            if (token.empty()) continue;

            size_t colon = token.find(':');
            if (colon == std::string::npos) {
                ROS_WARN("Malformed entry (missing priority): %s", token.c_str());
                continue;
            }

            std::string name = token.substr(0, colon);
            double prio = atof(token.substr(colon + 1).c_str());

            auto it = ctx.landmark_table.find(name);
            if (it == ctx.landmark_table.end()) {
                ROS_WARN("Unknown waypoint: %s", name.c_str());
                continue;
            }

            Landmark lm = it->second;
            lm.priority = prio;
            selected.push_back(lm);
        }

        if (selected.empty()) {
            ROS_WARN("Tour selection was empty after parsing.");
            return;
        }

        // Reset pause and set timing info for tour
        ctx.paused_by_timeout = false;
        ctx.tour_time_budget = time_budget;
        ctx.tour_start_time = ros::Time::now().toSec();
        ctx.tour_active = true;

        if (ctx.has_current_pose) {
            ctx.start_pose = {ctx.current_x, ctx.current_y};
            ctx.has_start_pose = true;
        } else {
            ctx.has_start_pose = false;
            ROS_WARN("Starting pose not yet known; return-to-start will be unavailable.");
        }

        // Build a time-constrained list of landmarks
        ctx.plan = buildTimeConstrainedPlan(selected, time_budget);
        ctx.current_plan_index = 0;

        ROS_INFO("Tour plan built with %lu points.", static_cast<unsigned long>(ctx.plan.size()));

        publishPlan();
        ROS_INFO("Published full tour plan to /tour_plan_path.");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        ctx.current_speed = std::sqrt(vx*vx + vy*vy);

        ctx.current_x = msg->pose.pose.position.x;
        ctx.current_y = msg->pose.pose.position.y;
        ctx.has_current_pose = true;
    }

    void logCallback(const rosgraph_msgs::Log::ConstPtr& msg)
    {
        ctx.last_log = msg->msg;
    }

    void spin()
    {
        ros::Rate rate(20);
        while (ros::ok())
        {
            ros::spinOnce();

            for (auto& b : behaviors)
                b->run();

            rate.sleep();
        }
    }

private:
    Context ctx;
    std::vector<std::unique_ptr<Behavior>> behaviors;

    void publishPlan()
    {
        nav_msgs::Path p;
        p.header.frame_id = "map";
        p.header.stamp = ros::Time::now();

        for (const auto& pt : ctx.plan)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = p.header;
            ps.pose.position.x = pt.x;
            ps.pose.position.y = pt.y;
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0;
            p.poses.push_back(ps);
        }

        ctx.plan_pub.publish(p);
    }

    void stopRobot()
    {
        ROS_WARN("Stopping robot: cancelling move_base goals and publishing zero velocity.");

        // Cancel move_base goals
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        if (!ac.waitForServer(ros::Duration(1.0))) {
            ROS_WARN("stopRobot: move_base server not available within timeout, sending zero cmd_vel anyway.");
        } else {
            ac.cancelAllGoals();
        }

        // Publish zero velocity
        geometry_msgs::Twist stop_twist;
        stop_twist.linear.x = 0.0;
        stop_twist.linear.y = 0.0;
        stop_twist.linear.z = 0.0;
        stop_twist.angular.x = 0.0;
        stop_twist.angular.y = 0.0;
        stop_twist.angular.z = 0.0;
        ctx.cmd_vel_pub.publish(stop_twist);
    }

    std::vector<PointFt> buildTimeConstrainedPlan(const std::vector<Landmark>& selected,
                                                  double time_budget)
    {
        if (selected.empty()) {
            return {};
        }

        // Determine maximum priority
        double max_prio = selected[0].priority;
        for (const auto& lm : selected) {
            if (lm.priority > max_prio) {
                max_prio = lm.priority;
            }
        }

        // Split into must-hit and optional waypoints
        std::vector<Landmark> must_hit;
        std::vector<Landmark> optional;
        must_hit.reserve(selected.size());
        optional.reserve(selected.size());

        for (const auto& lm : selected) {
            if (lm.priority >= max_prio) {
                must_hit.push_back(lm);
            } else {
                optional.push_back(lm);
            }
        }

        // Sort optional by descending priority
        std::sort(optional.begin(), optional.end(),
                  [](const Landmark& a, const Landmark& b) {
                      return a.priority > b.priority;
                  });

        // Assume a reasonable speed if not moving
        double est_speed = ctx.current_speed;
        if (est_speed < 0.05) {
            est_speed = 0.4;  // m/s
        }

        auto time_between = [&](const PointFt& a, const PointFt& b) {
            double d = dist(a.x, a.y, b.x, b.y);
            return d / est_speed;
        };

        std::vector<Landmark> chosen;

        double total_time = 0.0;
        PointFt last_point;

        bool have_last = false;
        if (ctx.has_current_pose) {
            last_point = {ctx.current_x, ctx.current_y};
            have_last = true;
        }

        // Always include must-hit landmarks (highest priority) ignoring time budget
        for (const auto& lm : must_hit) {
            if (have_last && time_budget > 0.0) {
                total_time += time_between(last_point, lm.location);
            }
            chosen.push_back(lm);
            last_point = lm.location;
            have_last = true;
        }

        // Add optional landmarks while staying under budget
        if (time_budget > 0.0) {
            for (const auto& lm : optional) {
                if (!have_last) {
                    last_point = lm.location;
                    chosen.push_back(lm);
                    have_last = true;
                    continue;
                }

                double dt = time_between(last_point, lm.location);
                if (total_time + dt <= time_budget) {
                    chosen.push_back(lm);
                    total_time += dt;
                    last_point = lm.location;
                } else {
                    ROS_WARN("Skipping waypoint (priority %.2f) due to time limit.", lm.priority);
                }
            }
        } else {
            // No budget: include everything
            for (const auto& lm : optional) {
                chosen.push_back(lm);
            }
        }

        if (chosen.empty()) {
            // Fallback: at least visit the first selected
            chosen.push_back(selected.front());
        }

        return two_opt_path(chosen);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "major_project");
    Bot bot;
    bot.spin();
    return 0;
}
