#ifndef __FRAME_CONVERTER__
#define __FRAME_CONVERTER__

#include <cmath>
#include <vector>
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "utils/common_struct.hpp"

enum class TOF_SIDE
{
    LEFT,
    RIGHT,
    BOTH
};

enum class ROW_NUMBER
{
    FIRST,
    SECOND,
    THIRD,
    FOURTH
};

class FrameConverter
{
public:
    FrameConverter();
    ~FrameConverter();

    std::vector<tPoint> transformTofSensor2RobotFrame(const std::vector<tPoint> &input_points,
                                                      bool isLeft, 
                                                      double rotation_yaw,
                                                      double rotation_pitch,
                                                      tPoint translation);
    std::vector<tPoint> transformCameraSensor2RobotFrame(const std::vector<tPoint> &input_points,
                                                         tPoint translation);
    std::vector<tPoint> transformCliffSensor2RobotFrame(robot_custom_msgs::msg::BottomIrData::SharedPtr msg,
                                                        std::vector<tPoint> &sensor_positions);
    std::vector<tPoint> transformRobot2GlobalFrame(const std::vector<tPoint> &input_points,
                                                   tPose robot_pose);
private:
};

#endif // FRAME_CONVERTER