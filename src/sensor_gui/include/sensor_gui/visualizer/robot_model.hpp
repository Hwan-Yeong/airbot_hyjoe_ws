#pragma once

#include <string>
#include <map>
#include <vector>
#include <GL/gl.h>

struct TfData; // Forward declaration

enum class ShapeType {
    CYLINDER,
    SPHERE
};

struct VisualSegment {
    std::string name;
    float x, y, z;
    float rx, ry, rz; // RPY in radians
    ShapeType shape_type = ShapeType::CYLINDER;
    float radius, length; // Assuming cylinder or sphere radius
    float cr, cg, cb, ca; // Color
};

class RobotModel {
public:
    RobotModel();
    ~RobotModel() = default;

    /**
     * @brief Load robot model from URDF file
     * @param path Path to URDF file
     */
    bool loadFromFile(const std::string& path);

    /**
     * @brief Get wheelbase extracted from URDF (distance between wheels)
     */
    float getWheelbase() const { return wheelbase_; }

    /**
     * @brief Get wheel radius extracted from URDF
     */
    float getWheelRadius() const { return wheel_radius_; }

    /**
     * @brief Render the entire robot model at the given base_link transform
     * @param base_tf The transform data for base_link
     * @param body_alpha Transparency for the robot body (0.0 to 1.0)
     */
    void draw(const TfData& base_tf, float body_alpha = 0.5f);

    static void drawCylinder(float radius, float length, float r, float g, float b, float a);
    static void drawSphere(float radius, float r, float g, float b, float a);

private:
    std::vector<VisualSegment> segments_;
    
    // Internal helper to find color by material name
    struct Material { float r, g, b, a; };
    std::map<std::string, Material> materials_;

    // Parsed physics parameters
    float wheelbase_ = 0.38f; // Default fallback
    float wheel_radius_ = 0.045f; // Default fallback
};
