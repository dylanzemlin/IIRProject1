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

static double landmark_weight(Landmark from, Landmark to)
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
        size_t add_idx = (size_t)-1;

        for (size_t idx = 0; idx < points_count; idx++)
        {
            if (!mst[idx].in_mst && mst[idx].min_edge < min_edge)
            {
                min_edge = mst[idx].min_edge;
                add_idx = idx;
            }
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
    tour.reserve(points_count);
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

    tour.push_back(0);

    bool improved = true;
    while (improved)
    {
        improved = false;
        for (size_t i = 0; i < points_count - 1; i++)
        {
            for (size_t j = i + 2; j < points_count && j != i; j++)
            {
                Landmark a = points[tour[i]];
                Landmark b = points[tour[i+1]];
                Landmark c = points[tour[j]];
                Landmark d = points[tour[(j+1) % points_count]];

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
    out.reserve(points_count);
    for (size_t i = 0; i < tour.size(); i++)
        out.push_back(points[tour[i]].location);

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

    ROS_INFO("Loaded %lu waypoints from file.", table.size());
    return table;
}

struct Context {
    ros::NodeHandle nh;
    ros::Subscriber tour_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber log_sub;

    ros::Publisher plan_pub;

    std::unordered_map<std::string, Landmark> landmark_table;

    std::vector<PointFt> plan;
    size_t current_plan_index = 0;

    double current_speed = 0.0;
    std::string last_log;
};

class Behavior {
public:
    explicit Behavior(Context& c) : ctx(c) {}
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
        ROS_INFO("Connected.");
    }

    bool run() override
    {
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
            ROS_WARN("Failed: %s", st.toString().c_str());

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

        behaviors.push_back(std::make_unique<MoveBaseBehavior>(ctx));
    }

    void tourCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("UI Requested Tour: %s", msg->data.c_str());

        std::stringstream ss(msg->data);
        std::string token;
        std::vector<Landmark> selected;

        while (std::getline(ss, token, ','))
        {
            token.erase(remove_if(token.begin(), token.end(), ::isspace), token.end());

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
            ROS_WARN("Tour selection was empty.");
            return;
        }

        ctx.plan = two_opt_path(selected);
        ctx.current_plan_index = 0;

        ROS_INFO("Tour rebuilt with %lu points.", ctx.plan.size());

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
        ROS_INFO("Published full tour plan to /tour_plan_path.");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        ctx.current_speed = std::sqrt(vx*vx + vy*vy);
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "major_project");
    Bot bot;
    bot.spin();
    return 0;
}
