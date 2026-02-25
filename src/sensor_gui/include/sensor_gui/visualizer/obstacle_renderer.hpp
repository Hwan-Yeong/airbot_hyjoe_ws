#pragma once

#include <QColor>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cmath>
#include <string>
#include <vector>

enum class ObstacleType { kBox, kCylinder, kCone };

struct SimObstacle {
    ObstacleType type = ObstacleType::kBox;
    float x = 0, y = 0, z = 0;
    float sx = 1, sy = 1, sz = 1;
    std::string name;
};

class IObstacleRenderer {
public:
    virtual ~IObstacleRenderer() = default;
    virtual void draw(const SimObstacle& ob, bool selected) = 0;
};

class BoxRenderer : public IObstacleRenderer {
public:
    void draw(const SimObstacle& ob, bool selected) override {
        glPushMatrix();
        glTranslatef(ob.x, ob.y, ob.z);
        
        QColor color = selected ? QColor("#ff4444") : QColor("#8888ff");
        glColor4f(color.redF(), color.greenF(), color.blueF(), selected ? 0.6f : 0.4f);
        
        float x1 = -ob.sx/2, x2 = ob.sx/2;
        float y1 = -ob.sy/2, y2 = ob.sy/2;
        float z1 = -ob.sz/2, z2 = ob.sz/2;

        glBegin(GL_QUADS);
        glVertex3f(x1, y1, z1); glVertex3f(x2, y1, z1); glVertex3f(x2, y2, z1); glVertex3f(x1, y2, z1);
        glVertex3f(x1, y1, z2); glVertex3f(x2, y1, z2); glVertex3f(x2, y2, z2); glVertex3f(x1, y2, z2);
        glVertex3f(x1, y1, z1); glVertex3f(x2, y1, z1); glVertex3f(x2, y1, z2); glVertex3f(x1, y1, z2);
        glVertex3f(x1, y2, z1); glVertex3f(x2, y2, z1); glVertex3f(x2, y2, z2); glVertex3f(x1, y2, z2);
        glVertex3f(x1, y1, z1); glVertex3f(x1, y2, z1); glVertex3f(x1, y2, z2); glVertex3f(x1, y1, z2);
        glVertex3f(x2, y1, z1); glVertex3f(x2, y2, z1); glVertex3f(x2, y2, z2); glVertex3f(x2, y1, z2);
        glEnd();
        
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(selected ? 3.0f : 1.0f);
        glColor4f(0, 0, 0, 0.5f);
        glBegin(GL_QUADS);
        glVertex3f(x1, y1, z1); glVertex3f(x2, y1, z1); glVertex3f(x2, y2, z1); glVertex3f(x1, y2, z1);
        glVertex3f(x1, y1, z2); glVertex3f(x2, y1, z2); glVertex3f(x2, y2, z2); glVertex3f(x1, y2, z2);
        glEnd();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        
        glPopMatrix();
    }
};

class CylinderRenderer : public IObstacleRenderer {
public:
    void draw(const SimObstacle& ob, bool selected) override {
        glPushMatrix();
        glTranslatef(ob.x, ob.y, ob.z - ob.sz/2);
        QColor color = selected ? QColor("#ff4444") : QColor("#88ff88");
        glColor4f(color.redF(), color.greenF(), color.blueF(), 0.4f);
        
        float R = ob.sx;
        float H = ob.sz;
        int segments = 24;
        glBegin(GL_QUAD_STRIP);
        for (int i = 0; i <= segments; ++i) {
            float rad = (i * 2.0f * M_PI) / segments;
            glVertex3f(std::cos(rad) * R, std::sin(rad) * R, 0);
            glVertex3f(std::cos(rad) * R, std::sin(rad) * R, H);
        }
        glEnd();

        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0, 0, 0);
        for (int i = 0; i <= segments; ++i) {
            float rad = (i * 2.0f * M_PI) / segments;
            glVertex3f(std::cos(rad) * R, std::sin(rad) * R, 0);
        }
        glEnd();

        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0, 0, H);
        for (int i = 0; i <= segments; ++i) {
            float rad = (i * 2.0f * M_PI) / segments;
            glVertex3f(std::cos(rad) * R, std::sin(rad) * R, H);
        }
        glEnd();
        glPopMatrix();
    }
};

class ConeRenderer : public IObstacleRenderer {
public:
    void draw(const SimObstacle& ob, bool selected) override {
        glPushMatrix();
        glTranslatef(ob.x, ob.y, ob.z - ob.sz/2);
        QColor color = selected ? QColor("#ff4444") : QColor("#ffff88");
        glColor4f(color.redF(), color.greenF(), color.blueF(), 0.4f);
        
        float R = ob.sx;
        float H = ob.sz;
        int segments = 24;
        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0, 0, H);
        for (int i = 0; i <= segments; ++i) {
            float rad = (i * 2.0f * M_PI) / segments;
            glVertex3f(std::cos(rad) * R, std::sin(rad) * R, 0);
        }
        glEnd();
        glPopMatrix();
    }
};
