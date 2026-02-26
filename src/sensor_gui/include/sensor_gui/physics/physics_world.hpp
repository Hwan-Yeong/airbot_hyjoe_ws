#pragma once

#include <btBulletDynamicsCommon.h>
#include <memory>
#include <vector>
#include <map>
#include "sensor_gui/visualizer/obstacle_renderer.hpp"

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    void init();
    void stepSimulation(float dt);

    // Obstacle management
    void syncObstacles(std::vector<SimObstacle>& obs);
    void clearObstacles();

    // Robot management
    void setRobotPose(float x, float y, float z, float roll, float pitch, float yaw);
    void getRobotPose(float& x, float& y, float& z, float& roll, float& pitch, float& yaw);
    void setRobotVelocity(float linear_x, float angular_z);
    
    void getWheelPoses(float& lx, float& ly, float& lz, float& lqx, float& lqy, float& lqz, float& lqw,
                       float& rx, float& ry, float& rz, float& rqx, float& rqy, float& rqz, float& rqw);

    // Dynamic Physics Parameters
    std::map<std::string, float> getPhysicsParams() const;
    void setPhysicsParams(const std::map<std::string, float>& params);

    // Create/update Robot
    void initRobot(float wheel_radius = 0.045f, float wheelbase = 0.38f, float mass = 10.0f);

private:
    std::unique_ptr<btDefaultCollisionConfiguration> collision_config_;
    std::unique_ptr<btCollisionDispatcher> dispatcher_;
    std::unique_ptr<btBroadphaseInterface> overlapping_pair_cache_;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
    std::unique_ptr<btDiscreteDynamicsWorld> dynamics_world_;

    // Ground plane
    btRigidBody* ground_body_ = nullptr;

    // Obstacles
    struct PhysObstacle {
        btRigidBody* body;
        std::string name;
    };
    std::vector<PhysObstacle> phys_obstacles_;

    // Robot Chassis & Wheels
    btRigidBody* robot_body_ = nullptr;
    btRigidBody* left_wheel_body_ = nullptr;
    btRigidBody* right_wheel_body_ = nullptr;
    
    btHingeConstraint* left_hinge_ = nullptr;
    btHingeConstraint* right_hinge_ = nullptr;

    float wheel_radius_ = 0.045f;
    float wheelbase_ = 0.38f;
    
    // Configurable Physics Parameters
    float robot_mass_ = 10.0f;
    float wheel_mass_ = 2.0f;
    float wheel_friction_ = 1.0f;
    float damping_ = 0.5f;
    float max_motor_impulse_ = 100.0f;
    float solver_iterations_ = 50.0f; // float for map compatibility
    
    float robot_linear_vel_x_ = 0.0f;
    float robot_angular_vel_z_ = 0.0f;
};
