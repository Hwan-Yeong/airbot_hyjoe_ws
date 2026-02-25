#include <tinyxml2.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include "sensor_gui/visualizer/robot_model.hpp"
#include "sensor_gui/visualizer/point_cloud_visualizer.hpp"

RobotModel::RobotModel() {
    // Basic materials from provided URDF as defaults
    materials_["light_black"] = {0.4f, 0.4f, 0.4f, 1.0f};
    materials_["dark"]        = {0.3f, 0.3f, 0.3f, 1.0f};
    materials_["white"]       = {1.0f, 1.0f, 1.0f, 1.0f};
}

bool RobotModel::loadFromFile(const std::string& path) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load URDF: " << path << std::endl;
        return false;
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("robot");
    if (!root) return false;

    // 1. Parse Materials
    for (auto* m = root->FirstChildElement("material"); m; m = m->NextSiblingElement("material")) {
        const char* name = m->Attribute("name");
        if (!name) continue;
        auto* color = m->FirstChildElement("color");
        if (color) {
            const char* rgba = color->Attribute("rgba");
            if (rgba) {
                float r, g, b, a;
                std::stringstream ss(rgba);
                ss >> r >> g >> b >> a;
                materials_[name] = {r, g, b, a};
            }
        }
    }

    struct LinkInfo {
        float ox = 0, oy = 0, oz = 0;
        float orx = 0, ory = 0, orz = 0;
        float radius = 0, length = 0;
        std::string material;
    };
    std::map<std::string, LinkInfo> link_map;

    // 2. Parse Links (Visuals)
    for (auto* l = root->FirstChildElement("link"); l; l = l->NextSiblingElement("link")) {
        const char* name = l->Attribute("name");
        if (!name) continue;

        auto* visual = l->FirstChildElement("visual");
        if (visual) {
            LinkInfo info;
            auto* origin = visual->FirstChildElement("origin");
            if (origin) {
                const char* xyz = origin->Attribute("xyz");
                if (xyz) { std::stringstream ss(xyz); ss >> info.ox >> info.oy >> info.oz; }
                const char* rpy = origin->Attribute("rpy");
                if (rpy) { std::stringstream ss(rpy); ss >> info.orx >> info.ory >> info.orz; }
            }
            auto* geometry = visual->FirstChildElement("geometry");
            if (geometry) {
                auto* cyl = geometry->FirstChildElement("cylinder");
                if (cyl) {
                    info.radius = cyl->FloatAttribute("radius");
                    info.length = cyl->FloatAttribute("length");
                }
            }
            auto* mat = visual->FirstChildElement("material");
            if (mat) {
                const char* mat_name = mat->Attribute("name");
                if (mat_name) info.material = mat_name;
            }
            link_map[name] = info;
        }
    }

    // 3. Parse Joints to build simple tree relative to base_link
    struct Joint { std::string parent, child; float x,y,z, rx, ry, rz; };
    std::vector<Joint> joints;
    for (auto* j = root->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
        const char* pname = j->FirstChildElement("parent")->Attribute("link");
        const char* cname = j->FirstChildElement("child")->Attribute("link");
        Joint joint;
        joint.parent = pname; joint.child = cname;
        auto* origin = j->FirstChildElement("origin");
        if (origin) {
            const char* xyz = origin->Attribute("xyz");
            if (xyz) { std::stringstream ss(xyz); ss >> joint.x >> joint.y >> joint.z; }
            const char* rpy = origin->Attribute("rpy");
            if (rpy) { std::stringstream ss(rpy); ss >> joint.rx >> joint.ry >> joint.rz; }
        }
        joints.push_back(joint);
    }

    // Simplified: Trace from base_link. For a real URDF we'd use a tree.
    // For this robot: base_link -> wheels, base_link -> base_scan -> lidars
    segments_.clear();

    // Map to store global transform relative to base_link
    struct Transform { float x=0,y=0,z=0, rx=0,ry=0,rz=0; };
    std::map<std::string, Transform> global_tfs;
    global_tfs["base_link"] = {0,0,0,0,0,0};

    // Recursive or iterative joint resolution (simple since URDF is flat-ish)
    bool changed = true;
    while (changed) {
        changed = false;
        for (const auto& j : joints) {
            if (global_tfs.count(j.parent) && !global_tfs.count(j.child)) {
                Transform pt = global_tfs[j.parent];
                // Simple addition for RPY (not correct for general 3D but works for this URDF)
                global_tfs[j.child] = {pt.x + j.x, pt.y + j.y, pt.z + j.z, pt.rx + j.rx, pt.ry + j.ry, pt.rz + j.rz};
                changed = true;
            }
        }
    }

    for (auto const& [name, tf] : global_tfs) {
        if (link_map.count(name)) {
            LinkInfo li = link_map[name];
            VisualSegment seg;
            seg.name = name;
            // Cumulative offset
            seg.x = tf.x + li.ox; seg.y = tf.y + li.oy; seg.z = tf.z + li.oz;
            seg.rx = tf.rx + li.orx; seg.ry = tf.ry + li.ory; seg.rz = tf.rz + li.orz;
            seg.radius = li.radius; seg.length = li.length;
            
            if (materials_.count(li.material)) {
                auto m = materials_[li.material];
                seg.cr = m.r; seg.cg = m.g; seg.cb = m.b; seg.ca = m.a;
            } else {
                seg.cr = 0.5f; seg.cg = 0.5f; seg.cb = 0.5f; seg.ca = 1.0f;
            }
            segments_.push_back(seg);
        }
    }

    return true;
}

void RobotModel::draw(const TfData& base_tf, float body_alpha) {
    glPushMatrix();
    
    glTranslatef(base_tf.x, base_tf.y, base_tf.z);
    
    float x = base_tf.qx, y = base_tf.qy, z = base_tf.qz, w = base_tf.qw;
    float mat[16];
    mat[0]  = 1.0f - 2.0f * (y * y + z * z);
    mat[1]  = 2.0f * (x * y + z * w);
    mat[2]  = 2.0f * (x * z - y * w);
    mat[3]  = 0.0f;
    mat[4]  = 2.0f * (x * y - z * w);
    mat[5]  = 1.0f - 2.0f * (x * x + z * z);
    mat[6]  = 2.0f * (y * z + x * w);
    mat[7]  = 0.0f;
    mat[8]  = 2.0f * (x * z + y * w);
    mat[9]  = 2.0f * (y * z - x * w);
    mat[10] = 1.0f - 2.0f * (x * x + y * y);
    mat[11] = 0.0f;
    mat[12] = 0.0f; mat[13] = 0.0f; mat[14] = 0.0f; mat[15] = 1.0f;
    glMultMatrixf(mat);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for (const auto& seg : segments_) {
        glPushMatrix();
        glTranslatef(seg.x, seg.y, seg.z);
        // Simple RPY rotation (Z-Y-X or similar, URDF is rpy)
        glRotatef(seg.rz * 180.0f / M_PI, 0, 0, 1);
        glRotatef(seg.ry * 180.0f / M_PI, 0, 1, 0);
        glRotatef(seg.rx * 180.0f / M_PI, 1, 0, 0);
        
        float alpha = seg.ca;
        if (seg.name == "base_link") alpha *= body_alpha;
        
        if (seg.radius > 0) {
            drawCylinder(seg.radius, seg.length, seg.cr, seg.cg, seg.cb, alpha);
        }
        glPopMatrix();
    }

    glDisable(GL_BLEND);
    glPopMatrix();
}
void RobotModel::drawCylinder(float radius, float length, float r, float g, float b, float a) {
    glColor4f(r, g, b, a);
    int slices = 16;
    float halfLen = length / 2.0f;
    
    // Sides
    glBegin(GL_TRIANGLE_STRIP);
    for (int i = 0; i <= slices; ++i) {
        float angle = i * 2.0f * M_PI / slices;
        float x = std::cos(angle) * radius;
        float y = std::sin(angle) * radius;
        glVertex3f(x, y, -halfLen);
        glVertex3f(x, y, halfLen);
    }
    glEnd();
    
    // End caps
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, 0, halfLen);
    for (int i = 0; i <= slices; ++i) {
        float angle = i * 2.0f * M_PI / slices;
        glVertex3f(std::cos(angle) * radius, std::sin(angle) * radius, halfLen);
    }
    glEnd();
    
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, 0, -halfLen);
    for (int i = 0; i <= slices; ++i) {
        float angle = i * 2.0f * M_PI / slices;
        glVertex3f(std::cos(angle) * radius, -std::sin(angle) * radius, -halfLen);
    }
    glEnd();
}
