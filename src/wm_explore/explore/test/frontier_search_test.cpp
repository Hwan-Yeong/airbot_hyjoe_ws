#include "nav_msgs/msg/occupancy_grid.hpp"
#include "explore/frontier_search.hpp"

#include <gtest/gtest.h>


using namespace explore;

TEST(FrontierSearchTest, EasyTest)
{
    FrontierSearch();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}