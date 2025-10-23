#ifndef __STATES_H__
#define __STATES_H__

enum class MAIN_STATE
{
    IDLE,
    STOP,
    BACK,
    FRONT,
    WAIT,
    TURN,
};

enum class SUB_STATE
{
    IDLE,
    DROP_OFF,
    ONE_D_TOF,
    DROP_IR,
    COLLISION,
    LIDAR_FRONT,
    LIDAR_BACK,
    ESCAPE_TURN,
    ESCAPE_MOVE,
    NO_LOCAL_PLAN,
    COLLISION_BACK,
    FORCE_ESCAPE_WAIT,
    FORCE_ESCAPE_TURN,
    FORCE_ESCAPE_MOVE,
};

// enum class MAPPING_STATE
// {
//     PREPARATION_STATE = 0,
//     WAIT_START_COMMAND_STATE = 1,
//     STOP_STATE = 2,
//     EXPLORING_STATE = 3,
//     COMPLETE_STATE = 4,
//     ERROR_STATE = 5
// };

inline std::string enumToString(MAIN_STATE in)
{
    std::string out;
    switch (in)
    {
        case MAIN_STATE::IDLE:
            out = std::string("IDLE");
            break;
        case MAIN_STATE::STOP:
            out = std::string("STOP");
            break;
        case MAIN_STATE::BACK:
            out = std::string("BACK");
            break;
        case MAIN_STATE::FRONT:
            out = std::string("FRONT");
            break;
        case MAIN_STATE::WAIT:
            out = std::string("WAIT");
            break;
        case MAIN_STATE::TURN:
            out = std::string("TURN");
            break;
    }
    return out;
};

inline std::string enumToString(SUB_STATE in)
{
    std::string out;
    switch (in)
    {
        case SUB_STATE::IDLE:
            out = std::string("IDLE");
            break;
        case SUB_STATE::DROP_OFF:
            out = std::string("DROP_OFF");
            break;
        case SUB_STATE::ONE_D_TOF:
            out = std::string("ONE_D_TOF");
            break;
        case SUB_STATE::DROP_IR:
            out = std::string("DROP_IR");
            break;
        case SUB_STATE::COLLISION:
            out = std::string("COLLISION");
            break;
        case SUB_STATE::LIDAR_FRONT:
            out = std::string("LIDAR_FRONT");
            break;
        case SUB_STATE::LIDAR_BACK:
            out = std::string("LIDAR_BACK");
            break;
        case SUB_STATE::ESCAPE_TURN:
            out = std::string("ESCAPE_TURN");
            break;
        case SUB_STATE::ESCAPE_MOVE:
            out = std::string("ESCAPE_MOVE");
            break;
        case SUB_STATE::NO_LOCAL_PLAN:
            out = std::string("NO_LOCAL_PLAN");
            break;
        case SUB_STATE::COLLISION_BACK:
            out = std::string("COLLISION_BACK");
            break;
        case SUB_STATE::FORCE_ESCAPE_WAIT:
            out = std::string("FORCE_ESCAPE_WAIT");
            break;
        case SUB_STATE::FORCE_ESCAPE_TURN:
            out = std::string("FORCE_ESCAPE_TURN");
            break;
        case SUB_STATE::FORCE_ESCAPE_MOVE:
            out = std::string("FORCE_ESCAPE_MOVE");
            break;
    }
    return out;
};

// inline std::string enumToString(MAPPING_STATE in)
// {
//     std::string out;
//     switch (in)
//     {
//         case MAPPING_STATE::PREPARATION_STATE:
//             out = std::string("PREPARATION_STATE");
//             break;
//         case MAPPING_STATE::WAIT_START_COMMAND_STATE:
//             out = std::string("WAIT_START_COMMAND_STATE");
//             break;
//         case MAPPING_STATE::STOP_STATE:
//             out = std::string("STOP_STATE");
//             break;
//         case MAPPING_STATE::EXPLORING_STATE:
//             out = std::string("EXPLORING_STATE");
//             break;
//         case MAPPING_STATE::COMPLETE_STATE:
//             out = std::string("COMPLETE_STATE");
//             break;
//         case MAPPING_STATE::ERROR_STATE:
//             out = std::string("ERROR_STATE");
//             break;
//     }
//     return out;
// };
#endif
