#include <gtest/gtest.h>
#include "legoslam/common_include.h"
#include "legoslam/algorithm.h"

TEST(LEGOSLAMTest, Triangulation) {
    Vec3 pt_world(30, 20, 10), pt_world_estimated;
    std::vector<SE3> poses{
            SE3(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 0, 0)),
            SE3(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, -10, 0)),
            SE3(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 10, 0))};
    VecVec3 points;

    for (auto &pose : poses) {
        Vec3 pc = pose * pt_world;
        pc /= pc[2];
        points.push_back(pc);
    }

    EXPECT_TRUE(legoslam::triangulation(poses, points, pt_world_estimated));
    EXPECT_NEAR(pt_world[0], pt_world_estimated[0], 0.01);
    EXPECT_NEAR(pt_world[1], pt_world_estimated[1], 0.01);
    EXPECT_NEAR(pt_world[2], pt_world_estimated[2], 0.01);
}

/// main
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
