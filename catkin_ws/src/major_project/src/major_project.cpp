#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
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

struct PointFt {
    double x{0.0}, y{0.0};
};

/*
    dist: standard distance function, calculates the distance between two points
*/
static double dist(double x1, double y1, double x2, double y2) {
    const double dx = x2 - x1, dy = y2 - y1;
    return std::hypot(dx, dy);
}

/*
   Returns the optimal path amongst passed in points according to a custom 2-opt algo

   First we construct an initial tour according to a MST with a link back to the start,
   then we do a pass of 2-opt optimization

   Assumes the first point in the list is the origin
*/
static
std::vector<PointFt> two_opt_path(const std::vector<PointFt> &points)
{
  size_t points_count = points.size();

  //
  // Build a Min Spanning Tree for the points, using prim's algo
  // Use that to give us a good initial tour to then do 3-opt refinement
  //

  // Metadata we need to keep track of for each point for MST algo
  struct MST_Point
  {
    // For building MST
    bool   in_mst   = false;
    double min_edge = std::numeric_limits<double>::infinity(); // Don't know it yet
    size_t parent_index = 0; // Everyone starts connected to origin

    // For traversing
    std::vector <size_t> neighbor_indices; // For adjacency stuff when we make our initial route from mst
    bool visited = false;
  };

  std::vector<MST_Point> mst_points(points_count);

  mst_points[0].in_mst = true; // Origin is always in mst
  for (size_t i = 1; i < points_count; i++)
  {
    // Remember: from the origin
    mst_points[i].min_edge = dist(points[0].x, points[0].y, points[i].x, points[i].y);
  }

  // Ok now we want to actually build the tree
  for (size_t edge_count = 0; edge_count < points_count - 1; edge_count++)
  {
    // Pick the cheapest edge we haven't already added
    double min_edge = std::numeric_limits<double>::infinity();
    size_t add_idx = (size_t) -1; // overflow to max since size_t is unsigned...
    for (size_t point_idx = 0; point_idx < points_count; point_idx++)
    {
      if (!mst_points[point_idx].in_mst && mst_points[point_idx].min_edge < min_edge)
      {
        min_edge = mst_points[point_idx].min_edge;
        add_idx = point_idx;
      }
    }

    assert(add_idx != (size_t)-1 && "Uh oh, not able to find an edge");

    // Add this new cheapest edge
    mst_points[add_idx].in_mst = true;

    // And add adjacency info
    size_t parent = mst_points[add_idx].parent_index;
    mst_points[parent].neighbor_indices.push_back(add_idx);
    mst_points[add_idx].neighbor_indices.push_back(parent);

    // Now update everyone not in our tree with distances from the most recently added point of our MST
    for (size_t point_idx = 0; point_idx < points_count; point_idx++)
    {
      if (!mst_points[point_idx].in_mst)
      {
        double new_dist = dist(points[add_idx].x, points[add_idx].y, points[point_idx].x, points[point_idx].y);
        if (new_dist < mst_points[point_idx].min_edge)
        {
          mst_points[point_idx].min_edge = new_dist;
          mst_points[point_idx].parent_index = add_idx;
        }
      }
    }
  }

  // Now we construct our initial tour from the MST, ie we traverse it with DFS
  std::vector<size_t> tour; // Indices, as always
  tour.reserve(points_count);

  std::stack<size_t> stack;
  stack.push(0);
  while (!stack.empty())
  {
    size_t current = stack.top();
    stack.pop();

    MST_Point *point = &mst_points[current];
    if (point->visited)
    {
      continue;
    }

    // Add it to tour
    point->visited = true;
    tour.push_back(current);

    // Add children
    for (size_t neighbor_index = point->neighbor_indices.size(); neighbor_index-- > 0;)
    {
      size_t neighbor = point->neighbor_indices[neighbor_index];

      if (!mst_points[neighbor].visited)
      {
        stack.push(neighbor);
      }
    }
  }

  // Add the origin to the end of tour... 2-opt will hopefully make this better in case that's a really bad choice
  tour.push_back(0);

  double tour_distance = 0.0;
  for (size_t i = 0; i < tour.size() - 1; i++)
  {
    PointFt a = points[tour[i]];
    PointFt b = points[tour[i + 1]];
    tour_distance += dist(a.x, a.y, b.x, b.y);
  }

  // 2-Opt refinement: take 2 edges and see if swapping would improve the tour, keep doing this until we don't see any improvement

  bool improved = true;
  while (improved) {
    improved = false;

    for (size_t i = 0; i < points_count - 1; i++)
    {
      for (size_t j = i + 2; j < points_count && j != i; j++)
      {
        PointFt a = points[tour[i]];
        PointFt b = points[tour[i + 1]];
        PointFt c = points[tour[j]];
        PointFt d = points[tour[(j + 1) % points_count]];

        // Before and after swap
        double before = dist(a.x, a.y, b.x, b.y) + dist(c.x, c.y, d.x, d.y);
        double after  = dist(a.x, a.y, c.x, c.y) + dist(b.x, b.y, d.x, d.y);


        // If we see improvement, do the swap, but we need to reverse the edges in between too, to make the tour make sense
        if (after < before)
        {
          size_t left = i + 1, right = j;
          while (left < right)
          {
            std::swap(tour[left], tour[right]);
            left++;
            right--;
          }

          improved = true;
        }
      }
    }
  }

  tour_distance = 0.0;
  for (size_t i = 0; i < tour.size() - 1; i++)
  {
    PointFt a = points[tour[i]];
    PointFt b = points[tour[i + 1]];
    tour_distance += dist(a.x, a.y, b.x, b.y);
  }

  // Yay! We are finished and grab the actual points from our optimized tour
  std::vector<PointFt> result;
  result.reserve(points_count);

  for (size_t i = 0; i < tour.size(); i++)
  {
    result.push_back(points[tour[i]]);
  }

  return result;
}

/*
    Contains useful information for each behavior to run which includes things like
    - plan_: A queue of points the robot is going to attempt to follow
*/
struct Context
{
  ros::NodeHandle handle_{"~"};

  // Map landmark names to points
  std::unordered_map<std::string, PointFt> landmark_table;

  std::vector<PointFt> plan;
  size_t current_plan_index; // Which point we are heading to
};

/*
    Behavior: A standard Behavior class that has a single run function, the run function
        is run in order and returns true or false depending on whether or not
        it made any updates to the robots state
*/
class Behavior
{
  public:
    explicit Behavior(Context& ctx) : ctx_(ctx) {}
    virtual ~Behavior() {}
    virtual bool run() = 0;

  protected:
    Context& ctx_;
};

class MoveBaseBehavior : public Behavior
{
  public:

    // Oh, brother
    using MoveBaseClient =
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

    explicit MoveBaseBehavior(Context& ctx)
      : Behavior(ctx),
      ac_("move_base", true)
    {
      ROS_INFO("Waiting for move_base...");
      ac_.waitForServer();
      ROS_INFO("Connected to move_base");
    }

    bool run() override
    {
      if (ctx_.current_plan_index >= ctx_.plan.size())
      {
        return false;
      }

      const PointFt& p = ctx_.plan[ctx_.current_plan_index];

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = p.x;
      goal.target_pose.pose.position.y = p.y;
      goal.target_pose.pose.orientation.w = 1.0;

      ROS_INFO("Sending goal %zu (%.2f, %.2f)",
               ctx_.current_plan_index, p.x, p.y);

      ac_.sendGoal(goal);
      ac_.waitForResult(); // blocking?

      auto state = ac_.getState();
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Reached waypoint %zu", ctx_.current_plan_index);
      }
      else
      {
        ROS_WARN("Failed waypoint %zu: %s",
                 ctx_.current_plan_index, state.toString().c_str());
      }

      ctx_.current_plan_index++;
      return true;
    }

  private:
    MoveBaseClient ac_;
};

class Bot
{
  public:
    Bot() : ctx_()
    {
      // Just dummy stuff for now...
      ctx_.landmark_table =
      {
        {"A", {0, 0}},
        {"B", {5, 1}},
        {"C", {10, 0}},
        {"D", {12, 4}},
        {"E", {10, 8}},
        {"F", {5, 10}},
        {"G", {0, 8}},
        {"H", {-2, 4}},
        {"I", {3, 3}},
        {"J", {7, 2}},
        {"K", {8, 6}},
        {"L", {6, 9}},
        {"M", {2, 7}},
        {"N", {4, 5}},
        {"O", {6, 4}},
        {"P", {1, 2}}
      };

      // FIXME: Hard-coded.
      std::vector<std::string> wish_tour_landmarks = { "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", };

      // Grab the actual points
      std::vector<PointFt> wish_tour_points;
      wish_tour_points.reserve(wish_tour_landmarks.size());
      for (auto& name : wish_tour_landmarks)
      {
        auto bucket = ctx_.landmark_table.find(name);
        if (bucket != ctx_.landmark_table.end())
        {
          wish_tour_points.push_back(bucket->second);
        }
      }

      // Use cool algorithm for good path
      ctx_.plan = two_opt_path(wish_tour_points);
      ctx_.current_plan_index = 0;

      behaviors_.push_back(std::make_unique<MoveBaseBehavior>(ctx_));
    }

    void spin()
    {
      ros::Rate rate(20.0);
      while (ros::ok()) {
        ros::spinOnce();

        for (auto &b : behaviors_) {
            b->run();
        }

        rate.sleep();
      }
    }

  private:
    Context ctx_;

    ros::Publisher pub_cmd_;

    std::vector<std::unique_ptr<Behavior>> behaviors_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "major_project");
  Bot bot;
  bot.spin();
  return 0;
}
