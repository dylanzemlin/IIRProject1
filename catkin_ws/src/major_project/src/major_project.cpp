#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <regex>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

/*
    The following three data structures are used to store target points, waypoints, and tasks (which are just collections of points)
    They are used for the path planning and path following algorithms
    - PointFt: A point that is going to be used in the path planning step
    - Waypoint: A point that is going to be followed at some point and has been path planned on
    - Task: A starting point and ending point for a line as well as metadata about whether or not its been followed
*/
struct PointFt {
    double x{0.0}, y{0.0};
};
struct Waypoint {
    double x_m{0.0}, y_m{0.0};
    int task_id{-1};
    bool is_start{false};
};
struct Task {
    PointFt start_ft;
    PointFt goal_ft;
    bool start_done{false};
    bool goal_done{false};
};

/*
    Contains useful information for each behavior to run which includes things like
    - plan_: A queue of points the robot is going to attempt to follow
    - progress_last_dist_ (and similar): Variables used to track if the robot is actually making progress towards the waypoint or if its stuck
    - tasks_: A list of current tasks the robot is following
*/
struct Context {
    ros::NodeHandle handle_{"~"};

    nav_msgs::Odometry odom_;
    bool have_odom_{false};

    geometry_msgs::Twist current_command_;

    // navigation and monitoring
    std::deque<Waypoint> plan_;
    bool navigating_{false};
    double progress_last_dist_{std::numeric_limits<double>::infinity()};
    ros::Time progress_last_improve_{0};

    // tasks
    std::vector<Task> tasks_;
    bool have_plan_{false};

    double start_x_m_{0.0}, start_y_m_{0.0}, start_yaw_{0.0};
    bool start_pose_set_{false};
};

/*
    yawFrom: Converts from a nav_msgs Odometry message to a single double (yaw)
*/
static double yawFrom(const nav_msgs::Odometry& od) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(od.pose.pose.orientation, q);
    double r, p, y;
    tf::Matrix3x3(q).getRPY(r, p, y);
    return y;
}

/*
    normAngle: normalizes an angle between 0 and 2PI (subtracts down or adds up to it)
*/
static double normAngle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

/*
    dist: standard distance function, calculates the distance between two points
*/
static double dist(double x1, double y1, double x2, double y2) {
    const double dx = x2 - x1, dy = y2 - y1;
    return std::hypot(dx, dy);
}

/*
    clamp: standard clamping function, clamps a value between a lower bound and upper bound
*/
static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

/*
    Behavior: A standard Behavior class that has a single run function, the run function
        is run in order and returns true or false depending on whether or not
        it made any updates to the robots state
*/
class Behavior {
   public:
    explicit Behavior(Context& ctx) : ctx_(ctx) {}
    virtual ~Behavior() {}
    virtual bool run() = 0;

   protected:
    Context& ctx_;
};

/*
    TaskManagerBehavior: Handles building a plan if the robot does not currently have any plans
        will also reset variables like last progress made, etc. 
*/
class TaskManagerBehavior : public Behavior {
   public:
    explicit TaskManagerBehavior(Context& ctx) : Behavior(ctx) {}

    bool run() override {
        if (!ctx_.have_plan_ && !ctx_.tasks_.empty() && ctx_.have_odom_) {
            std::vector<Waypoint> ordered_points = createMostlyOptimalPath();
            ctx_.plan_.clear();

            for (auto& w : ordered_points) {
                ctx_.plan_.push_back(w);
            }
            ctx_.have_plan_ = true;
            ctx_.navigating_ = !ctx_.plan_.empty();
            ctx_.progress_last_dist_ = std::numeric_limits<double>::infinity();
            ctx_.progress_last_improve_ = ros::Time::now();

            ROS_INFO_STREAM("[Planner] Built plan with " << ctx_.plan_.size() << " waypoints.");
            return false;
        }
        return false;
    }

    /*
        createMostlyOptimalPath: as the name implies, creates a "mostly optimal path" by performing a very scuffed
            waypoint solver which just constantly takes the next shortest waypoint over and over again
            until it creates a "solved" path. Certainly not fully optimal, but mostly optimal enough  
    */
    std::vector<Waypoint> createMostlyOptimalPath() {
        struct Node {
            int task_id;
            bool is_start;
            double x_m;
            double y_m;
        };
        std::vector<Node> remaining;

        for (size_t i = 0; i < ctx_.tasks_.size(); ++i) {
            const auto& t = ctx_.tasks_[i];
            remaining.push_back(Node{static_cast<int>(i), true, t.start_ft.x * 0.3048, t.start_ft.y * 0.3048});
            remaining.push_back(Node{static_cast<int>(i), false, t.goal_ft.x * 0.3048, t.goal_ft.y * 0.3048});
        }

        std::vector<Waypoint> result;
        std::vector<bool> start_done(ctx_.tasks_.size(), false);
        std::vector<bool> goal_done(ctx_.tasks_.size(), false);

        auto current_x = ctx_.have_odom_ ? ctx_.odom_.pose.pose.position.x : ctx_.start_x_m_;
        auto current_y = ctx_.have_odom_ ? ctx_.odom_.pose.pose.position.y : ctx_.start_y_m_;
        while (!remaining.empty()) {
            double best_d = std::numeric_limits<double>::infinity();
            int best_idx = -1;
            for (int i = 0; i < static_cast<int>(remaining.size()); ++i) {
                const auto& n = remaining[i];
                if (!n.is_start && !start_done[n.task_id]) {
                    continue;
                }

                double d = dist(current_x, current_y, n.x_m, n.y_m);
                if (d < best_d) {
                    best_d = d;
                    best_idx = i;
                }
            }
            if (best_idx < 0) {
                break;
            }

            const auto chosen = remaining[best_idx];
            remaining.erase(remaining.begin() + best_idx);
            result.push_back(Waypoint{chosen.x_m, chosen.y_m, chosen.task_id, chosen.is_start});
            if (chosen.is_start) {
                start_done[chosen.task_id] = true;
            } else {
                goal_done[chosen.task_id] = true;
            }

            current_x = chosen.x_m;
            current_y = chosen.y_m;
        }
        return result;
    }
};

/*
    NavigatorBehavior: Handles getting the current plan and the next available waypoint and attempting to navigate to it.
        That includes handling figuring out the desired yaw/speed, determing if there is any forward progress being made, etc.
*/
class NavigatorBehavior : public Behavior {
   public:
    explicit NavigatorBehavior(Context& ctx) : Behavior(ctx) {}

    bool run() override {
        if (!ctx_.have_plan_ || !ctx_.navigating_ || ctx_.plan_.empty() || !ctx_.have_odom_) {
            return false;
        }

        Waypoint& target = ctx_.plan_.front();
        const double rx = ctx_.odom_.pose.pose.position.x;
        const double ry = ctx_.odom_.pose.pose.position.y;
        const double yaw = yawFrom(ctx_.odom_);

        const double dx = target.x_m - rx;
        const double dy = target.y_m - ry;
        const double dist_now = std::hypot(dx, dy);
        const double desired_yaw = std::atan2(dy, dx);
        const double yaw_err = normAngle(desired_yaw - yaw);

        // Check if we have many any forward progress
        ros::Time now = ros::Time::now();
        if (dist_now + 1e-3 < ctx_.progress_last_dist_) {
            ctx_.progress_last_dist_ = dist_now;
            ctx_.progress_last_improve_ = now;
        }

        // If we have made it within 1ft of the target waypoint the robot has made it and we can move on
        if (dist_now <= 1 * 0.3048) {
            ROS_INFO_STREAM("[Monitor] Reached waypoint for task "
                            << target.task_id
                            << (target.is_start ? " (START)" : " (DEST)")
                            << " at (" << target.x_m / 0.3048 << " ft, "
                            << target.y_m / 0.3048 << " ft)");

            // mark task state as completed
            if (target.is_start) {
                ctx_.tasks_[target.task_id].start_done = true;
            } else {
                ctx_.tasks_[target.task_id].goal_done = true;
            }

            // pop the front of the plan so we can move onto the next waypoint
            ctx_.plan_.pop_front();
            ctx_.progress_last_dist_ = std::numeric_limits<double>::infinity();
            ctx_.progress_last_improve_ = now;

            // if the plan is empty we are done :)
            if (ctx_.plan_.empty()) {
                ctx_.navigating_ = false;
                ctx_.have_plan_ = false;
                ctx_.tasks_.clear();
                ROS_INFO("[Monitor] All waypoints complete. Ready for a new set of tasks.");
            }

            return true;
        }

        // check if we have many any progress, if not handle it
        if ((now - ctx_.progress_last_improve_).toSec() > 6) {
            ROS_WARN_STREAM("[Monitor] Stuck before reaching waypoint for task " << target.task_id << (target.is_start ? " (START)" : " (DEST)"));
            
            // if its a start node remove that entire task, otherwise just that node
            if (target.is_start) {
                removeTaskDestinationFromPlan(target.task_id);
                ctx_.plan_.pop_front();
                ctx_.tasks_[target.task_id].start_done = true;
            } else {
                ctx_.plan_.pop_front();
                ctx_.tasks_[target.task_id].goal_done = true;
            }

            ctx_.progress_last_dist_ = std::numeric_limits<double>::infinity();
            ctx_.progress_last_improve_ = now;

            // if the plan is empty, we are done :)
            if (ctx_.plan_.empty()) {
                ctx_.navigating_ = false;
                ctx_.have_plan_ = false;
                ctx_.tasks_.clear();
                ROS_INFO("[Monitor] Plan exhausted after recovery. Ready for new tasks.");
            }
            return true;
        }

        double w_cmd = clamp(1.8 * yaw_err, -1.2, 1.2);
        double v_cmd = 0.0;

        if (std::fabs(yaw_err) > M_PI / 8.0) {
            v_cmd = 0.0;
        } else {
            v_cmd = clamp(0.8 * dist_now, 0.0, 0.3);
        }

        geometry_msgs::Twist cmd;
        cmd.linear.x = v_cmd;
        cmd.angular.z = w_cmd;
        ctx_.current_command_ = cmd;
        return true;
    }

   private:
    /*
        removeTaskDestinationFromPlan: Given a task_id, remove that entire task from the current plan
    */
    void removeTaskDestinationFromPlan(int task_id) {
        std::deque<Waypoint> new_plan;
        for (auto& w : ctx_.plan_) {
            if (w.task_id == task_id && !w.is_start) {
                continue;
            }

            new_plan.push_back(w);
        }
        ctx_.plan_.swap(new_plan);
        ctx_.tasks_[task_id].goal_done = true;
        ROS_INFO_STREAM("[Planner] Removed DEST of task " << task_id << " from plan due to start failure.");
    }
};

/*
    PathFollowerBehavior: Just simply runs the navigation behavior, was added just incase other functionality was added later on
        before the navigation run portion
*/
class PathFollowerBehavior : public Behavior {
   public:
    explicit PathFollowerBehavior(Context& ctx) : Behavior(ctx), nav_(ctx) {}
    bool run() override { return nav_.run(); }

   private:
    NavigatorBehavior nav_;
};

class Bot {
   public:
    Bot() : ctx_() {
        pub_cmd_ = ctx_.handle_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
        sub_odom_ = ctx_.handle_.subscribe("/odom", 1, &Bot::odomCallback, this);
        sub_tasks_ = ctx_.handle_.subscribe("/task_lines", 10, &Bot::taskLineCallback, this);

        behaviors_.emplace_back(new TaskManagerBehavior(ctx_));
        behaviors_.emplace_back(new PathFollowerBehavior(ctx_));

        ROS_INFO("[Init] Publish each line like '((2, 3), (9, 8))' to /task_lines. Send an empty line to start.");
    }

    void spin() {
        ros::Rate rate(20.0);
        while (ros::ok()) {
            ros::spinOnce();
            ctx_.current_command_ = geometry_msgs::Twist();

            for (auto& b : behaviors_) {
                if (b->run()) {
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
    ros::Subscriber sub_odom_, sub_tasks_;
    std::vector<std::unique_ptr<Behavior>> behaviors_;
    std::vector<std::string> pending_task_lines_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        ctx_.odom_ = *msg;
        if (!ctx_.have_odom_) {
            ctx_.have_odom_ = true;
            if (!ctx_.start_pose_set_) {
                ctx_.start_x_m_ = ctx_.odom_.pose.pose.position.x;
                ctx_.start_y_m_ = ctx_.odom_.pose.pose.position.y;
                ctx_.start_yaw_ = yawFrom(ctx_.odom_);
                ctx_.start_pose_set_ = true;
            }
        }
    }

    void taskLineCallback(const std_msgs::String::ConstPtr& msg) {
        const std::string line = trim(msg->data);
        if (line.empty()) {
            if (pending_task_lines_.empty()) {
                return;
            }
            parsePendingTasks();
            pending_task_lines_.clear();

            if (!ctx_.navigating_) {
                ctx_.have_plan_ = false;
            }

            return;
        }
        pending_task_lines_.push_back(line);
    }

    static std::string trim(const std::string& s) {
        const auto wsfront = std::find_if_not(
            s.begin(), s.end(), [](int c) { return std::isspace(c); });
        const auto wsback = std::find_if_not(s.rbegin(), s.rend(), [](int c) {
                                return std::isspace(c);
                            }).base();
        if (wsback <= wsfront) return std::string();
        return std::string(wsfront, wsback);
    }

    void parsePendingTasks() {
        // i love regex
        std::regex rx(R"(\(\(\s*([\-+]?\d+(\.\d+)?)\s*,\s*([\-+]?\d+(\.\d+)?)\s*\)\s*,\s*\(\s*([\-+]?\d+(\.\d+)?)\s*,\s*([\-+]?\d+(\.\d+)?)\s*\)\s*\))");
        int added = 0;
        for (const auto& ln : pending_task_lines_) {
            std::smatch m;
            if (std::regex_search(ln, m, rx)) {
                Task t;
                t.start_ft.x = std::stod(m[1]);
                t.start_ft.y = std::stod(m[3]);
                t.goal_ft.x = std::stod(m[5]);
                t.goal_ft.y = std::stod(m[7]);
                ctx_.tasks_.push_back(t);
                ++added;
            }
        }

        ROS_INFO_STREAM("[Tasks] Parsed " << added << " tasks.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "project2_turtlebot");
    Bot bot;
    bot.spin();
    return 0;
}