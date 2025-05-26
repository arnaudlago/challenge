#include <iostream>
#include <string>

#include <gtest/gtest.h>
#include <pure_pursuit.h>

float lookahead_distance_ = 5.0;
float wheelbase_ = 1.75;

TEST(PurePursuitTest, SetPath) {
    PurePursuit pp(lookahead_distance_, wheelbase_);

    std::vector<Point> path = {{0, 0}, {1, 0}, {2, 0}};
    EXPECT_TRUE(pp.SetPath(path));

    // Check if invalid paths are correctly identified.
    std::vector<Point> empty_path;
    EXPECT_FALSE(pp.IsPathValid(empty_path));

   // Check if invalid paths are correctly identified.
    std::vector<Point> wrongPath = {{0, 0}, {0, 1}, {1, 1}, {0, 2}};
    EXPECT_FALSE(pp.IsPathValid(empty_path));
}

TEST(PurePursuitTest, ComputeSteeringAngle) {
    PurePursuit pp(lookahead_distance_, wheelbase_);
    int idx = 0;
    float x = 0;
    float y = 0;
    float yaw = 0;
    float max_steering_angle = 0.5;

    //Test to compute steering angle while there is no path
    EXPECT_THROW(pp.ComputeSteeringAngle(idx, x, y, yaw, max_steering_angle), std::invalid_argument);

    std::vector<Point> path = {{0, 0}, {1, 0}, {2, 0}};
    pp.SetPath(path);

    //Compute steering anngle is vehicle is perfectly alligned on trajectory
    EXPECT_FLOAT_EQ(pp.ComputeSteeringAngle(idx, x, y, yaw, max_steering_angle), 0);


    //Test path length from an out of range index
    idx = 10;
    EXPECT_THROW(pp.ComputeSteeringAngle(idx, x, y, yaw, max_steering_angle), std::out_of_range);
}

TEST(PurePursuitTest, ComputeSpeed) {
    PurePursuit pp(lookahead_distance_, wheelbase_);
    int idx = 0;
    float x = 0;
    float y = 0;
    float deceleration = 3.0;
    float max_speed = 10;

    //Test to compute steering angle while there is no path
    EXPECT_THROW(pp.ComputeSpeed(idx, x, y, deceleration, max_speed), std::invalid_argument);

    std::vector<Point> path = {{0, 0}, {1, 0}, {20, 0}};
    pp.SetPath(path);

    //Compute steering anngle is vehicle is perfectly alligned on trajectory
    EXPECT_FLOAT_EQ(pp.ComputeSpeed(idx, x, y, deceleration, max_speed), 10);

    //Test path length from an out of range index
    idx = 10;
    EXPECT_THROW(pp.ComputeSpeed(idx, x, y, deceleration, max_speed), std::out_of_range);
}

TEST(PurePursuitTest, ComputeRemainingDistance) {
    PurePursuit pp(lookahead_distance_, wheelbase_);

    //Test to compute remaining distance while there is no path
    EXPECT_THROW(pp.ComputeRemainingDistance(0), std::invalid_argument);

    std::vector<Point> path = {{0, 0}, {0, 1}, {0, 2}};
    pp.SetPath(path);

    //Test path length from first point
    EXPECT_FLOAT_EQ(pp.ComputeRemainingDistance(0), 2.0);

    //Test path length from last point
    EXPECT_FLOAT_EQ(pp.ComputeRemainingDistance(2), 0.0);

    //Test path length from an out of range index
    EXPECT_THROW(pp.ComputeRemainingDistance(10), std::out_of_range);
}

TEST(PurePursuitTest, FindNearestPathPointIndex) {
    PurePursuit pp(lookahead_distance_, wheelbase_);
    int idx = 0;
    float x = 0;
    float y = 0;

    //Test to compute remaining distance while there is no path
    EXPECT_THROW(pp.FindNearestPathPointIndex(idx, x, y), std::invalid_argument);

    std::vector<Point> path = {{0, 0}, {10, 0}, {20, 0}};
    pp.SetPath(path);

    //Test path length from first point
    EXPECT_FLOAT_EQ(pp.FindNearestPathPointIndex(idx, x, y), 0);

    //Test path length from last point
    x = 8;
    EXPECT_FLOAT_EQ(pp.FindNearestPathPointIndex(idx, x, y), 1);

    //Test path length from last point
    x = 16;
    EXPECT_FLOAT_EQ(pp.FindNearestPathPointIndex(idx, x, y), 2);

    //Test path length from an out of range index
    idx = 100;
    EXPECT_THROW(pp.FindNearestPathPointIndex(idx, x, y), std::out_of_range);
}

TEST(PurePursuitTest, FindTargetPoint) {
    PurePursuit pp(lookahead_distance_, wheelbase_);
    int idx = 0 ;
    float x = 0;
    float y = 0;
    //Test to compute remaining distance while there is no path
    EXPECT_THROW(pp.FindTargetPoint(idx, x, y), std::invalid_argument);

    std::vector<Point> path = {{0, 0}, {10, 0}, {20, 0}, {30, 0}};
    pp.SetPath(path);

    //Test path length from first point
    EXPECT_EQ(pp.FindTargetPoint(idx, x, y), 1);

    idx = 1;
    x = 10.0;
    EXPECT_EQ(pp.FindTargetPoint(idx, x, y), 2);
  

    //Test path length from an out of range index
    idx = 10;
    EXPECT_THROW(pp.FindTargetPoint(idx, x, y), std::out_of_range);
}

int main(int argc, char * argv[]) {
  ::testing::InitGoogleTest(&argc,argv);

  return RUN_ALL_TESTS();
}