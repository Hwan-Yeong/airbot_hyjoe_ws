#pragma once

#include <string>
#include <map>
#include <vector>
#include <GL/gl.h>

struct TfData; // Forward declaration

struct VisualSegment {
    std::string name;
    float x, y, z;
    float rx, ry, rz; // RPY in radians
    float radius, length; // Assuming cylinder for now
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
     * @brief Render the entire robot model at the given base_link transform
     * @param base_tf The transform data for base_link
     * @param body_alpha Transparency for the robot body (0.0 to 1.0)
     */
    void draw(const TfData& base_tf, float body_alpha = 0.5f);

    static void drawCylinder(float radius, float length, float r, float g, float b, float a);

private:
    std::vector<VisualSegment> segments_;
    
    // Internal helper to find color by material name
    struct Material { float r, g, b, a; };
    std::map<std::string, Material> materials_;
};
