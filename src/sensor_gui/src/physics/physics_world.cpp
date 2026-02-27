#include "sensor_gui/physics/physics_world.hpp"
#include <iostream>

PhysicsWorld::PhysicsWorld() {
}

PhysicsWorld::~PhysicsWorld() {
    clearObstacles();
    
    if (dynamics_world_) {
        // Remove Constraints
        if (left_hinge_) dynamics_world_->removeConstraint(left_hinge_);
        if (right_hinge_) dynamics_world_->removeConstraint(right_hinge_);

        auto safelyRemoveBody = [&](btRigidBody* body) {
            if (body) {
                dynamics_world_->removeRigidBody(body);
                if (body->getMotionState()) delete body->getMotionState();
                if (body->getCollisionShape()) {
                    btCollisionShape* shape = body->getCollisionShape();
                    if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
                        btCompoundShape* compound = static_cast<btCompoundShape*>(shape);
                        std::vector<btCollisionShape*> unique_shapes;
                        for (int i = compound->getNumChildShapes() - 1; i >= 0; i--) {
                            btCollisionShape* childShape = compound->getChildShape(i);
                            compound->removeChildShapeByIndex(i);
                            if (std::find(unique_shapes.begin(), unique_shapes.end(), childShape) == unique_shapes.end()) {
                                unique_shapes.push_back(childShape);
                            }
                        }
                        for (auto* uniqueShape : unique_shapes) {
                            delete uniqueShape;
                        }
                    }
                    delete shape;
                }
                delete body;
            }
        };

        safelyRemoveBody(left_wheel_body_);
        safelyRemoveBody(right_wheel_body_);
        safelyRemoveBody(caster_front_body_);
        safelyRemoveBody(caster_rear_body_);
        safelyRemoveBody(robot_body_);
        safelyRemoveBody(ground_body_);
    }

    if (left_hinge_) delete left_hinge_;
    if (right_hinge_) delete right_hinge_;
    if (caster_front_spring_) delete caster_front_spring_;
    if (caster_rear_spring_) delete caster_rear_spring_;
}

void PhysicsWorld::init() {
    collision_config_ = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher_ = std::make_unique<btCollisionDispatcher>(collision_config_.get());
    overlapping_pair_cache_ = std::make_unique<btDbvtBroadphase>();
    solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
    dynamics_world_ = std::make_unique<btDiscreteDynamicsWorld>(
        dispatcher_.get(), overlapping_pair_cache_.get(), solver_.get(), collision_config_.get());

    dynamics_world_->setGravity(btVector3(0, 0, -9.81f));
    dynamics_world_->getSolverInfo().m_numIterations = static_cast<int>(solver_iterations_); // Increase solver stability

    // Create ground
    btCollisionShape* ground_shape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
    btDefaultMotionState* ground_motion_state = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo ground_info(0, ground_motion_state, ground_shape, btVector3(0, 0, 0));
    ground_info.m_restitution = 0.0f; // Prevent micro-bouncing
    ground_info.m_friction = 1.0f;
    ground_info.m_rollingFriction = 0.1f; // Add rolling friction to ground
    ground_body_ = new btRigidBody(ground_info);
    dynamics_world_->addRigidBody(ground_body_);
}

void PhysicsWorld::stepSimulation(float dt) {
    if (dynamics_world_) {
        if (robot_body_ && left_hinge_ && right_hinge_) {
            float v_left = robot_linear_vel_x_ - robot_angular_vel_z_ * wheelbase_ / 2.0f;
            float v_right = robot_linear_vel_x_ + robot_angular_vel_z_ * wheelbase_ / 2.0f;

            // Wake up wheels and chassis only if we want to move
            if (std::abs(v_left) > 0.001f || std::abs(v_right) > 0.001f || std::abs(robot_angular_vel_z_) > 0.001f) {
                robot_body_->activate(true);
                left_wheel_body_->activate(true);
                right_wheel_body_->activate(true);
            }

            // Wheel targets
            float w_left = -v_left / wheel_radius_;
            float w_right = -v_right / wheel_radius_;

            // Enable Motors and set target velocity
            float max_motor_impulse = max_motor_impulse_; // Configurable torque
            
            if (std::abs(v_left) < 0.001f && std::abs(v_right) < 0.001f && std::abs(robot_angular_vel_z_) < 0.001f) {
                // Apply Parking Brake / High Damping when idle
                left_hinge_->enableAngularMotor(true, 0.0f, max_motor_impulse * 5.0f);
                right_hinge_->enableAngularMotor(true, 0.0f, max_motor_impulse * 5.0f);
                
                // For perfectly stopping Bullet bodies, we MUST clear all forces and clamp velocities to exact zero.
                robot_body_->setLinearVelocity(btVector3(0, 0, robot_body_->getLinearVelocity().z())); // Preserve gravity Z
                robot_body_->setAngularVelocity(btVector3(0, 0, 0));
                robot_body_->clearForces();
                
                left_wheel_body_->setLinearVelocity(btVector3(0, 0, left_wheel_body_->getLinearVelocity().z()));
                left_wheel_body_->setAngularVelocity(btVector3(0, 0, 0));
                left_wheel_body_->clearForces();

                right_wheel_body_->setLinearVelocity(btVector3(0, 0, right_wheel_body_->getLinearVelocity().z()));
                right_wheel_body_->setAngularVelocity(btVector3(0, 0, 0));
                right_wheel_body_->clearForces();

                if (caster_front_body_) {
                    caster_front_body_->setLinearVelocity(btVector3(0, 0, caster_front_body_->getLinearVelocity().z()));
                    caster_front_body_->setAngularVelocity(btVector3(0, 0, 0));
                    caster_front_body_->clearForces();
                }
                if (caster_rear_body_) {
                    caster_rear_body_->setLinearVelocity(btVector3(0, 0, caster_rear_body_->getLinearVelocity().z()));
                    caster_rear_body_->setAngularVelocity(btVector3(0, 0, 0));
                    caster_rear_body_->clearForces();
                }

                // Also let them sleep naturally by allowing deactivation
            } else {
                left_hinge_->enableAngularMotor(true, w_left, max_motor_impulse);
                right_hinge_->enableAngularMotor(true, w_right, max_motor_impulse);
            }
        }

        dynamics_world_->stepSimulation(dt, 10);
    }
}

void PhysicsWorld::clearObstacles() {
    if (!dynamics_world_) return;
    for (auto& po : phys_obstacles_) {
        dynamics_world_->removeRigidBody(po.body);
        delete po.body->getMotionState();
        delete po.body->getCollisionShape();
        delete po.body;
    }
    phys_obstacles_.clear();
}

void PhysicsWorld::syncObstacles(std::vector<SimObstacle>& obs) {
    if (!dynamics_world_) return;

    bool needs_rebuild = false;
    if (obs.size() != phys_obstacles_.size()) {
        needs_rebuild = true;
    } else {
        for (size_t i = 0; i < obs.size(); ++i) {
            if (obs[i].name != phys_obstacles_[i].name) {
                needs_rebuild = true;
                break;
            }
        }
    }

    if (needs_rebuild) {
        clearObstacles();
        for (const auto& o : obs) {
            btCollisionShape* shape = nullptr;
            float mass = 0.0f; 
            
            // Determine mass by object type and "name" heuristically, 
            // In a real scenario, mass would be a field in SimObstacle
            if (o.type == ObstacleType::kBox) {
                shape = new btBoxShape(btVector3(o.sx/2.0f, o.sy/2.0f, o.sz/2.0f));
                if (o.name == "bump" || o.name.find("wall") != std::string::npos || o.name.find("Static") != std::string::npos) {
                    mass = 0.0f; // static
                } else {
                    mass = o.sx * o.sy * o.sz * 100.0f; 
                }
            } else if (o.type == ObstacleType::kCylinder) {
                shape = new btCylinderShapeZ(btVector3(o.sx, o.sx, o.sz/2.0f));
                mass = 3.14159f * o.sx * o.sx * o.sz * 100.0f;
            } else if (o.type == ObstacleType::kSphere) {
                shape = new btSphereShape(o.sx);
                mass = 4.0f/3.0f * 3.14159f * o.sx * o.sx * o.sx * 100.0f;
            } else {
                shape = new btBoxShape(btVector3(o.sx/2.0f, o.sy/2.0f, o.sz/2.0f));
            }

            // Small obstacle hack: if it's very small, maybe it's just a light prop
            if (mass > 0.0f && mass < 1.0f) mass = 1.0f;

            btVector3 localInertia(0, 0, 0);
            if (mass > 0.0f) {
                shape->calculateLocalInertia(mass, localInertia);
            }

            btTransform startTransform;
            startTransform.setIdentity();
            startTransform.setOrigin(btVector3(o.x, o.y, o.z));
            startTransform.setRotation(btQuaternion(o.qx, o.qy, o.qz, o.qw));

            btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
            
            rbInfo.m_friction = 0.8f;
            rbInfo.m_restitution = 0.2f;

            btRigidBody* body = new btRigidBody(rbInfo);
            dynamics_world_->addRigidBody(body);

            PhysObstacle po;
            po.body = body;
            po.name = o.name;
            phys_obstacles_.push_back(po);
        }
    } else {
        // Sync Transform
        for (size_t i = 0; i < obs.size(); ++i) {
            btRigidBody* body = phys_obstacles_[i].body;
            btTransform transform;
            
            // If it's a dynamic body (mass > 0), Physics Engine overrides GUI
            if (body->getMass() > 0.0f && body->isActive()) {
                body->getMotionState()->getWorldTransform(transform);
                obs[i].x = transform.getOrigin().getX();
                obs[i].y = transform.getOrigin().getY();
                obs[i].z = transform.getOrigin().getZ();
                
                btQuaternion q = transform.getRotation();
                obs[i].qx = q.getX();
                obs[i].qy = q.getY();
                obs[i].qz = q.getZ();
                obs[i].qw = q.getW();
            } else {
                // If static, user might have dragged it in GUI, so GUI overrides Physics
                transform.setOrigin(btVector3(obs[i].x, obs[i].y, obs[i].z));
                transform.setRotation(btQuaternion(obs[i].qx, obs[i].qy, obs[i].qz, obs[i].qw));
                body->setWorldTransform(transform);
                if (body->getMotionState()) {
                    body->getMotionState()->setWorldTransform(transform);
                }
                body->activate(true);
            }
        }
    }
}

void PhysicsWorld::initRobot(float wheel_radius, float wheelbase, float mass) {
    if (robot_body_ || !dynamics_world_) return;

    wheel_radius_ = wheel_radius;
    wheelbase_ = wheelbase;

    // 1. Chassis (Box Only, no fixed casters anymore)
    float chassis_x = 0.38f;
    float chassis_y = wheelbase_ - 0.05f; // slightly inside wheels
    float chassis_z = 0.15f; 
    float clearance = 0.03f; // ground clearance

    btCompoundShape* chassis_compound = new btCompoundShape();
    
    // Main Body Box
    btBoxShape* box_shape = new btBoxShape(btVector3(chassis_x/2.0f, chassis_y/2.0f, chassis_z/2.0f));
    btTransform box_trans;
    box_trans.setIdentity();
    chassis_compound->addChildShape(box_trans, box_shape);
    
    // Rigid casters (frictionless spheres attached to chassis to prevent dipping/penetration)
    float caster_x_offset = 0.135f;
    float caster_radius = 0.015f;
    // z = -(clearance + chassis_z/2.0f) + (caster_radius - wheel_radius_) 
    //   = -(0.03 + 0.075) + (0.015 - 0.05) = -0.105 - 0.035 = -0.140f
    float caster_z_offset = -0.140f; 

    btSphereShape* caster_shape = new btSphereShape(caster_radius);
    
    btTransform front_caster_trans;
    front_caster_trans.setIdentity();
    front_caster_trans.setOrigin(btVector3(caster_x_offset, 0.0f, caster_z_offset));
    chassis_compound->addChildShape(front_caster_trans, caster_shape);

    btTransform rear_caster_trans;
    rear_caster_trans.setIdentity();
    rear_caster_trans.setOrigin(btVector3(-caster_x_offset, 0.0f, caster_z_offset));
    chassis_compound->addChildShape(rear_caster_trans, caster_shape);

    btVector3 localInertia(0, 0, 0);
    chassis_compound->calculateLocalInertia(mass, localInertia);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(0, 0, wheel_radius_ + clearance + chassis_z/2.0f));

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, chassis_compound, localInertia);
    rbInfo.m_friction = 0.0f; // Frictionless casters!
    rbInfo.m_rollingFriction = 0.0f;
    
    robot_body_ = new btRigidBody(rbInfo);
    robot_body_->setDamping(damping_, damping_); // Add damping to reduce idle sliding
    robot_body_->setActivationState(DISABLE_DEACTIVATION);
    
    // Additional friction to prevent body sliding over casters
    robot_body_->setFriction(0.8f);
    
    dynamics_world_->addRigidBody(robot_body_);


    // 2. Wheels
    auto createWheel = [&](float y_offset) -> btRigidBody* {
        // Bullet cylinder is oriented along Y axis if we use btCylinderShapeX/Y/Z. 
        // We want rolling along X, so cylinder axis should be Y. 
        // btCylinderShape usually aligns with given half-extents. Wait, Bullet has btCylinderShapeX/Y/Z.
        // btCylinderShapeX aligns with X. We want rotation axis to be Y, so btCylinderShapeY.
        
        float wheel_width = 0.04f;
        btCollisionShape* wheel_shape = new btCylinderShape(btVector3(wheel_radius_, wheel_width/2.0f, wheel_radius_)); 
        // Default btCylinderShape is along Y axis!

        btVector3 wheelInertia(0, 0, 0);
        wheel_shape->calculateLocalInertia(wheel_mass_, wheelInertia);

        btTransform wheelTrans;
        wheelTrans.setIdentity();
        // Position relative to center
        wheelTrans.setOrigin(btVector3(0.0f, y_offset, wheel_radius_));
        
        // Ensure cylinder axis is Y:
        btQuaternion q;
        q.setEuler(0, 0, M_PI/2.0f); // Roll 90 deg?
        // Actually, default btCylinderShape aligns with Y axis natively.
        // So a wheel rotating around Y will roll forward/backward along Z/X. We want X.
        
        btDefaultMotionState* motionState = new btDefaultMotionState(wheelTrans);
        btRigidBody::btRigidBodyConstructionInfo wInfo(wheel_mass_, motionState, wheel_shape, wheelInertia);
        // High lateral friction
        wInfo.m_friction = wheel_friction_;
        wInfo.m_rollingFriction = 0.05f;

        btRigidBody* wheel_body = new btRigidBody(wInfo);
        wheel_body->setDamping(damping_, damping_); // Add damping
        wheel_body->setActivationState(DISABLE_DEACTIVATION);
        
        // Ensure lateral friction is high enough
        wheel_body->setFriction(wheel_friction_);

        dynamics_world_->addRigidBody(wheel_body);
        return wheel_body;
    };

    left_wheel_body_ = createWheel(wheelbase_ / 2.0f);
    right_wheel_body_ = createWheel(-wheelbase_ / 2.0f);

    // 4. Constraints (Hinges for wheels)
    btVector3 parentAxis(0, 1, 0); // Rotation around Y axis
    btVector3 childAxis(0, 1, 0);

    btVector3 pivotInChassisLeft(0, wheelbase_ / 2.0f, -(clearance + chassis_z/2.0f));
    btVector3 pivotInWheelLeft(0, 0, 0);
    left_hinge_ = new btHingeConstraint(*robot_body_, *left_wheel_body_, pivotInChassisLeft, pivotInWheelLeft, parentAxis, childAxis);
    dynamics_world_->addConstraint(left_hinge_, true);

    btVector3 pivotInChassisRight(0, -wheelbase_ / 2.0f, -(clearance + chassis_z/2.0f));
    btVector3 pivotInWheelRight(0, 0, 0);
    right_hinge_ = new btHingeConstraint(*robot_body_, *right_wheel_body_, pivotInChassisRight, pivotInWheelRight, parentAxis, childAxis);
    dynamics_world_->addConstraint(right_hinge_, true);

    // Casters are now rigidly attached to the chassis compound shape, 
    // so we don't need independent spring bodies or constraints anymore.
}

void PhysicsWorld::setRobotPose(float x, float y, float z, float roll, float pitch, float yaw) {
    if (robot_body_ && left_wheel_body_ && right_wheel_body_) {
        // Disable motors temporarily
        if (left_hinge_) left_hinge_->enableAngularMotor(false, 0, 0);
        if (right_hinge_) right_hinge_->enableAngularMotor(false, 0, 0);

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(x, y, z));
        btQuaternion q;
        q.setEulerZYX(yaw, pitch, roll);
        transform.setRotation(q);

        robot_body_->setWorldTransform(transform);
        if (robot_body_->getMotionState()) {
            robot_body_->getMotionState()->setWorldTransform(transform);
        }
        
        robot_body_->setLinearVelocity(btVector3(0,0,0));
        robot_body_->setAngularVelocity(btVector3(0,0,0));
        robot_body_->clearForces();

        // Also place wheels precisely where they belong relative to chassis
        btTransform leftWheelTrans = transform;
        leftWheelTrans.getOrigin() += leftWheelTrans.getBasis().getColumn(1) * (wheelbase_ / 2.0f); // move Y
        leftWheelTrans.getOrigin() -= leftWheelTrans.getBasis().getColumn(2) * (z - wheel_radius_); // move down Z
        
        left_wheel_body_->setWorldTransform(leftWheelTrans);
        if (left_wheel_body_->getMotionState()) left_wheel_body_->getMotionState()->setWorldTransform(leftWheelTrans);
        left_wheel_body_->setLinearVelocity(btVector3(0,0,0));
        left_wheel_body_->setAngularVelocity(btVector3(0,0,0));
        left_wheel_body_->clearForces();
        
        btTransform rightWheelTrans = transform;
        rightWheelTrans.getOrigin() -= rightWheelTrans.getBasis().getColumn(1) * (wheelbase_ / 2.0f);
        rightWheelTrans.getOrigin() -= rightWheelTrans.getBasis().getColumn(2) * (z - wheel_radius_);
        
        right_wheel_body_->setWorldTransform(rightWheelTrans);
        if (right_wheel_body_->getMotionState()) right_wheel_body_->getMotionState()->setWorldTransform(rightWheelTrans);
        right_wheel_body_->setLinearVelocity(btVector3(0,0,0));
        right_wheel_body_->setAngularVelocity(btVector3(0,0,0));
        right_wheel_body_->clearForces();
        
        if (caster_front_body_) {
            btTransform frontCasterTrans = transform;
            frontCasterTrans.getOrigin() += frontCasterTrans.getBasis().getColumn(0) * 0.135f; // move X
            frontCasterTrans.getOrigin() -= frontCasterTrans.getBasis().getColumn(2) * (z - 0.015f); // move down Z to touch ground
            
            caster_front_body_->setWorldTransform(frontCasterTrans);
            if (caster_front_body_->getMotionState()) caster_front_body_->getMotionState()->setWorldTransform(frontCasterTrans);
            caster_front_body_->setLinearVelocity(btVector3(0,0,0));
            caster_front_body_->setAngularVelocity(btVector3(0,0,0));
            caster_front_body_->clearForces();
        }
        
        if (caster_rear_body_) {
            btTransform rearCasterTrans = transform;
            rearCasterTrans.getOrigin() -= rearCasterTrans.getBasis().getColumn(0) * 0.135f; // move X
            rearCasterTrans.getOrigin() -= rearCasterTrans.getBasis().getColumn(2) * (z - 0.015f); // move down Z
            
            caster_rear_body_->setWorldTransform(rearCasterTrans);
            if (caster_rear_body_->getMotionState()) caster_rear_body_->getMotionState()->setWorldTransform(rearCasterTrans);
            caster_rear_body_->setLinearVelocity(btVector3(0,0,0));
            caster_rear_body_->setAngularVelocity(btVector3(0,0,0));
            caster_rear_body_->clearForces();
        }
    }
}

void PhysicsWorld::getRobotPose(float& x, float& y, float& z, float& roll, float& pitch, float& yaw) {
    if (robot_body_) {
        btTransform transform;
        if (robot_body_->getMotionState()) {
            robot_body_->getMotionState()->getWorldTransform(transform);
        } else {
            transform = robot_body_->getWorldTransform();
        }
        
        x = transform.getOrigin().getX();
        y = transform.getOrigin().getY();
        z = transform.getOrigin().getZ();

        transform.getBasis().getEulerZYX(yaw, pitch, roll);
    }
}

void PhysicsWorld::getWheelPoses(float& lx, float& ly, float& lz, float& lqx, float& lqy, float& lqz, float& lqw,
                                 float& rx, float& ry, float& rz, float& rqx, float& rqy, float& rqz, float& rqw) {
    if (left_wheel_body_ && right_wheel_body_) {
        btTransform transL;
        if (left_wheel_body_->getMotionState()) {
            left_wheel_body_->getMotionState()->getWorldTransform(transL);
        } else {
            transL = left_wheel_body_->getWorldTransform();
        }
        lx = transL.getOrigin().getX();
        ly = transL.getOrigin().getY();
        lz = transL.getOrigin().getZ();
        btQuaternion qL = transL.getRotation();
        lqx = qL.getX(); lqy = qL.getY(); lqz = qL.getZ(); lqw = qL.getW();

        btTransform transR;
        if (right_wheel_body_->getMotionState()) {
            right_wheel_body_->getMotionState()->getWorldTransform(transR);
        } else {
            transR = right_wheel_body_->getWorldTransform();
        }
        rx = transR.getOrigin().getX();
        ry = transR.getOrigin().getY();
        rz = transR.getOrigin().getZ();
        btQuaternion qR = transR.getRotation();
        rqx = qR.getX(); rqy = qR.getY(); rqz = qR.getZ(); rqw = qR.getW();
    }
}


void PhysicsWorld::setRobotVelocity(float linear_x, float angular_z) {
    robot_linear_vel_x_ = linear_x;
    robot_angular_vel_z_ = angular_z;
}

std::map<std::string, float> PhysicsWorld::getPhysicsParams() const {
    std::map<std::string, float> params;
    params["robot_mass"] = robot_mass_;
    params["wheel_mass"] = wheel_mass_;
    params["wheel_friction"] = wheel_friction_;
    params["damping"] = damping_;
    params["max_motor_impulse"] = max_motor_impulse_;
    params["solver_iterations"] = solver_iterations_;
    params["suspension_stiffness"] = suspension_stiffness_;
    params["suspension_damping"] = suspension_damping_;
    params["caster_mass"] = caster_mass_;
    return params;
}

void PhysicsWorld::setPhysicsParams(const std::map<std::string, float>& params) {
    auto updateIfPresent = [&](const std::string& key, float& val) {
        if (params.count(key)) val = params.at(key);
    };

    updateIfPresent("robot_mass", robot_mass_);
    updateIfPresent("wheel_mass", wheel_mass_);
    updateIfPresent("wheel_friction", wheel_friction_);
    updateIfPresent("damping", damping_);
    updateIfPresent("max_motor_impulse", max_motor_impulse_);
    updateIfPresent("solver_iterations", solver_iterations_);
    updateIfPresent("suspension_stiffness", suspension_stiffness_);
    updateIfPresent("suspension_damping", suspension_damping_);
    updateIfPresent("caster_mass", caster_mass_);

    // Apply immediate changes where possible without respawning
    if (dynamics_world_) {
        dynamics_world_->getSolverInfo().m_numIterations = static_cast<int>(solver_iterations_);
    }

    if (robot_body_) {
        robot_body_->setDamping(damping_, damping_);
    }
    
    if (left_wheel_body_) {
        left_wheel_body_->setFriction(wheel_friction_);
        left_wheel_body_->setDamping(damping_, damping_);
    }
    if (right_wheel_body_) {
        right_wheel_body_->setFriction(wheel_friction_);
        right_wheel_body_->setDamping(damping_, damping_);
    }
    
    if (caster_front_body_) caster_front_body_->setDamping(0.1f, 0.1f);
    if (caster_rear_body_) caster_rear_body_->setDamping(0.1f, 0.1f);
    
    if (caster_front_spring_) {
        caster_front_spring_->setStiffness(2, suspension_stiffness_);
        caster_front_spring_->setDamping(2, suspension_damping_);
    }
    if (caster_rear_spring_) {
        caster_rear_spring_->setStiffness(2, suspension_stiffness_);
        caster_rear_spring_->setDamping(2, suspension_damping_);
    }
}
