#include "../Math.h"
#include "../GlobalDefs.h"
#include "../PhysicsEngine.h"
#include "../Robot.h"

#include <Math.h>
#include <vector>

class CustomRobot : public Robot {
public:

    std::vector<float> external_angles;
    bool angles_set = false;

    CustomRobot() { }

    void set_simplify_mode(int n) {
        simplifyMode(n);
    }

    // Methods for robot construction
    bodyID create_box(FLOAT_32 length, FLOAT_32 width, FLOAT_32 height, FLOAT_32 posX, FLOAT_32 posY, FLOAT_32 posZ,
                     FLOAT_32 mass, FLOAT_32 colorR, FLOAT_32 colorG, FLOAT_32 colorB) {
        return createBox(length, width, height, posX, posY, posZ, mass, colorR, colorG, colorB);
    }

    bodyID create_cylinder(FLOAT_32 startX, FLOAT_32 startY, FLOAT_32 startZ,
                          FLOAT_32 endX, FLOAT_32 endY, FLOAT_32 endZ,
                          FLOAT_32 radius, FLOAT_32 mass,
                          FLOAT_32 colorR, FLOAT_32 colorG, FLOAT_32 colorB) {
        return createCylinder(startX, startY, startZ, endX, endY, endZ, radius, mass, colorR, colorG, colorB);
    }

    jointID create_joint(bodyID body1, bodyID body2,
                        FLOAT_32 jointX, FLOAT_32 jointY, FLOAT_32 jointZ,
                        FLOAT_32 axisX, FLOAT_32 axisY, FLOAT_32 axisZ) {
        return createJoint(body1, body2, jointX, jointY, jointZ, axisX, axisY, axisZ);
    }

    bodyID create_point(FLOAT_32 x, FLOAT_32 y, FLOAT_32 z, FLOAT_32 mass,
                        FLOAT_32 red = 0.5f, FLOAT_32 green = 0.5f, FLOAT_32 blue = 0.5f) {
        return createPoint(x, y, z, mass, red, green, blue);
    }

    bodyID create_sphere(FLOAT_32 x, FLOAT_32 y, FLOAT_32 z, FLOAT_32 radius,
                         FLOAT_32 mass, FLOAT_32 red = 0.5f, FLOAT_32 green = 0.5f, FLOAT_32 blue = 0.5f) {
        return createSphere(x, y, z, radius, mass, red, green, blue);
    }

    bodyID create_ray(FLOAT_32 x1, FLOAT_32 y1, FLOAT_32 z1, FLOAT_32 x2, FLOAT_32 y2, FLOAT_32 z2,
                      FLOAT_32 red = 0.5f, FLOAT_32 green = 0.5f, FLOAT_32 blue = 0.5f) {
        return createRay(x1, y1, z1, x2, y2, z2, red, green, blue);
    }

    // finalizes robot construction (call after all parts and joints were created)
    void finalize_construction() {
        // initialize external_angles vector now, so its length matches the amount of joints
        external_angles.resize(jointIDcounter, 0.0f);

        for ( int id=0; id<getMaxBodyID(); id++ ){
			// --- lifting robot up that it falls down on the ground
			moveBody(id,0,VERTICAL_OFFSET,0);
		}

        finalizeConstruction();
    }

    // set_joint_angles to influence movement
    void set_joint_angles(const std::vector<float>& angles) {
        // copy given angles into external_angles vector
        for (int i = 0; i < std::min(external_angles.size(), angles.size()); i++) {
            external_angles[i] = angles[i];
        }
        angles_set = true;
    }

    // moves the robots joints
    void move(int counter) {
        if (angles_set == true) {
            // use angles from external_angles
            for (int i = 0; i < external_angles.size(); i++) {
                angle[i] = external_angles[i];
            }
        } else {
            // default behaviour if no external_angles were set
            int i = 0;
            angle[i++] = (float) (sin(counter / 33.0) / 2.0);
            angle[i++] = (float) (sin(counter / 33.0) / 2.0);
            angle[i++] = (float) (cos(counter / 33.0) / 2.0);
            angle[i++] = (float) (cos(counter / 33.0) / 2.0);
        }
    }
};