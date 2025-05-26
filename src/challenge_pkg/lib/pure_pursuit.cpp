#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <pure_pursuit.h>


// Simple class to implement Pure Pursuit controller
PurePursuit::PurePursuit(){}

PurePursuit::PurePursuit(float lookahead_distance, float wheelbase)
    : lookahead_distance_(lookahead_distance), wheelbase_(wheelbase), is_path_valid_(false) {}

// Method to set the path (list of waypoints)
bool PurePursuit::SetPath(const std::vector<Point>& path) 
{
    is_path_valid_ = IsPathValid(path);
    if (!is_path_valid_)
    {
        path_.clear();
        return false;
    }
    path_ = path;
    return true;
}


//check the path validity, the path is valid if
// -- contains at least 2 points
// -- the angle between 2 consecutive segments  < PI/4
bool PurePursuit::IsPathValid(const std::vector<Point>& path)
{
    if (path.empty() || path.size() < 2)
    {
        return false;
    }
    if (path.size() == 2)
    {
        return true;
    }
    
    //check angle between segments
    for (int i = 0; i < path.size() - 3; i++)
    {
        float angle = ComputeAngle(path[i], path[i + 1], path[i + 2]);
        if (fabs(angle) >= M_PI_4)
        {
            return false;
        }
    }


    return true;
}

// Method to calculate the steering angle for the current position
float PurePursuit::ComputeSteeringAngle(int idx, float x,float y,float yaw, float max_steering_angle) 
{
    if (!is_path_valid_)
    {
        throw std::invalid_argument( "The path is invalid" );
    }
    if (idx >= path_.size())
    {
        throw std::out_of_range( "Index out of range" );
    }

    auto target_idx = FindTargetPoint(idx, x, y);

    // Transform the lookahead point to the vehicle's coordinate frame
    auto dx = path_[target_idx].x - x;
    auto dy = path_[target_idx].y - y;

    auto x_local = cos(yaw) * dx + sin(yaw) * dy;
    auto y_local = -sin(yaw) * dx + cos(yaw) * dy;

    // Calculate the steering angle using the Pure Pursuit formula
    auto steering_angle = atan2(2.0 * wheelbase_ * y_local, pow(lookahead_distance_ , 2));
        

    auto max_steering_angle_abs = fabs(max_steering_angle);

    if (steering_angle > max_steering_angle_abs) 
    {
        steering_angle = max_steering_angle_abs;
    }
    else if (steering_angle < -max_steering_angle_abs) 
    {
        steering_angle = -max_steering_angle_abs;
    }

    return steering_angle;
}

// Method to calculate the vehicle speed base on remaining travel distance 
// This helps to smooth the deceleration
float PurePursuit::ComputeSpeed(int idx, float x,float y, float deceleration, float max_speed)
{
    auto remaining_distance = ComputeRemainingDistance(idx);

    float speed = std::sqrt(2.0 * std::abs(deceleration) * remaining_distance);

    return std::min(speed, max_speed);

}

float PurePursuit::ComputeRemainingDistance(const int idx)
{
    if (!is_path_valid_)
    {
        throw std::invalid_argument( "The path is invalid" );
    }
    if (idx >= path_.size())
    {
        throw std::out_of_range( "Index out of range" );
    }

    auto distance = 0.0;
    auto last_index = path_.size() - 1;
    if (idx == last_index)
    {
        return distance;
    }
    auto i = idx;
    while (i < last_index)
    {
        auto segment_length = PurePursuit::ComputeDistance(
            path_[i].x, path_[i].y, 
            path_[i + 1].x, path_[i + 1].y);
        distance += segment_length;
        i += 1;
    }
    return distance;
}

int PurePursuit::FindNearestPathPointIndex(const int idx, float x, float y)
{
    if (!is_path_valid_)
    {
        throw std::invalid_argument( "The path is invalid" );
    }
    if (idx >= path_.size())
    {
        throw std::out_of_range( "Index is out of range" );
    }

    auto nearest_index = 0;
    auto closest_dist = 10000.0;

    if (idx < 0 || idx == path_.size() - 1)
    {
        return path_.size() - 1;
    }

    for (auto i = idx; i < path_.size(); i++)
    {
        auto distance = ComputeDistance(x, y, path_[i].x, path_[i].y);
        if (i == 0) 
        {
            closest_dist = distance;
            nearest_index = 0;
        }
        else if (distance < closest_dist ) 
        {
            closest_dist = distance;
            nearest_index = i;
        }
    }
    return nearest_index;
}

// Find the nearest path point from the lookahead point to reach
int PurePursuit::FindTargetPoint(int idx, float x, float y)
{
    if (!is_path_valid_)
    {
        throw std::invalid_argument( "The path is invalid" );
    }
    if (idx >= path_.size())
    {
        throw std::out_of_range( "Index is out of range" );
    }

    auto i = idx;
    while (ComputeDistance(path_[i].x, path_[i].y, x, y) < lookahead_distance_) 
    {
        i += 1;
        if (i == path_.size()) 
        { 
            i -= 1;
            break; 
        }
    }
    return i;
}

float PurePursuit::ComputeDistance(float p1_x, float p1_y, float p2_x, float p2_y)
{
    auto dx = p1_x - p2_x;
    auto dy = p1_y - p2_y;
    return dx * dx + dy * dy;
}

float PurePursuit::NormalizeAngle(float angle) 
{
    while (angle > M_PI) 
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) 
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

float PurePursuit::ComputeAngle(const Point& p1, const Point& p2, const Point& p3)
{
    auto v1_x = p2.x - p1.x;
    auto v1_y = p2.y - p1.y;

    auto v2_x = p3.x - p2.x;
    auto v2_y = p3.y - p2.y;

    auto dot = v1_x * v2_x + v1_y  * v2_y;
    auto mag1 = std::sqrt(v1_x * v1_x + v1_y * v1_y);
    auto mag2 = std::sqrt(v2_x * v2_x + v2_y * v2_y);

    auto cosTheta = dot / (mag1 * mag2);

    return std::acos(cosTheta);
}



