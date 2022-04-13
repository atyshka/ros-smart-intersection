#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>

namespace smart_intersection
{

enum class Direction {
  UP,
  DOWN,
  LEFT,
  RIGHT
};

class Intersection
{
  public:
    Intersection(const tf2::Transform pose, double distance = 50.0, double speed = 10.0, double interval = 2.0);
    double distance_, speed_, interval_;
    tf2::Transform pose_;

    // key: time in ms, 4 bools for each time step for each direction
    std::map<uint64_t, std::array<bool, 4>> slotmap_;

    void getTrajectory(std::vector<geometry_msgs::PoseStamped>& result, const geometry_msgs::PoseStamped& pose, double speed, const Direction& direction);

  private:
    void generateTrajectory(std::vector<double>& pos_out, double pos_f, double vel_0, double vel_f, double end_time, int resolution);
};

} // namespace smart_intersection