#ifndef __FRAME_CONVERTER__
#define __FRAME_CONVERTER__

#include <cmath>
#include <vector>
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
                                                      tPoint translation);
    std::vector<tPoint> transformCameraSensor2RobotFrame(const std::vector<tPoint> &input_points,
                                                         tPoint translation);
    std::vector<tPoint> transformRobot2GlobalFrame(const std::vector<tPoint> &input_points,
                                                   tPose robot_pose);
private:
};

#endif // FRAME_CONVERTER