#pragma once

#include <vector>

struct Point {
    double x;
    double y;
};

class PurePursuit {
public:
    PurePursuit();

    PurePursuit(float lookahead_distance, float wheelbase);

    bool SetPath(const std::vector<Point>& path);
    
    float ComputeSteeringAngle(int idx, float x, float y, float yaw, float max_steering_angle);

    float ComputeSpeed(int idx, float x,float y, float deceleration, float max_speed);

    int FindNearestPathPointIndex(int idx, float x, float y);

    int FindTargetPoint(int idx, float x, float y);

    float ComputeRemainingDistance(int index);

    bool IsPathValid(const std::vector<Point>& path);

private:
    std::vector<Point> path_;
    float lookahead_distance_;
    float wheelbase_;
    bool is_path_valid_;

    float ComputeDistance(float p1_x, float p1_y, float p2_x, float p2_y);

    float ComputeAngle(const Point& p1, const Point& p2, const Point& p3);
    
    float NormalizeAngle(float angle);
};
