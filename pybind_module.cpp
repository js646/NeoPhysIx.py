#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h> // Für die Konvertierung von std::vector nach Python-Liste

#include <array>
#include <vector>
#include <memory>

#include "src/GlobalDefs.h"
#include "src/RigidBody.h"
#include "src/Robot.h"
#include "src/control.h"

namespace py = pybind11;

PYBIND11_MODULE(NeoPhysIx, m) {
    m.doc() = "NeoPhysIx binding module";

    // Enums for configurations
    py::enum_<EnvironmentType>(m, "EnvironmentType")
        .value("Mountain", EnvironmentType::Mountain)
        .value("Ramp", EnvironmentType::Ramp)
        .value("Plane", EnvironmentType::Plane)
        .value("Apartment", EnvironmentType::Apartment);

    py::enum_<RobotType>(m, "RobotType")
        .value("FourLegged", RobotType::FourLegged)
        .value("FourLeggedComparison", RobotType::FourLeggedComparison)
        .value("SixLegged", RobotType::SixLegged)
        .value("Humanoid", RobotType::Humanoid)
        .value("Wheeled", RobotType::Wheeled);


    // Getter methods provided by this module
    m.def("get_landscape", []() {
        return py::array_t<double>(
            {MAX_WORLD + 1, MAX_WORLD + 1}, // shape
            {sizeof(double) * (MAX_WORLD + 1), sizeof(double)}, // strides
            &landscape[0][0] // data pointer
        );
    }, "Get the landscape array");

    m.def("get_robots", []() {
        std::vector<Robot*> robot_list;
        for (const auto& robot : robots) {
            robot_list.push_back(robot.get());
        }
        return robot_list;
    }, "Get the list of robots");


    // Methods to interact with NeoPhysIx (provided in Control.cpp)
    m.def("set_environment", &set_environment, py::arg("env_type"), "Set the environment for the simulation");
    m.def("set_gravity", &set_gravity, py::arg("value"), "Set the gravity value");

    m.def("add_robots", &add_robots, py::arg("robot_type"), py::arg("robot_count"), "Add predefined robots to the simulation");
    m.def("add_custom_robot", &add_custom_robot, "Add a custom robot to the simulation");
    m.def("clear_robots", &add_custom_robot, "Clears the robots array");

    m.def("move_robot", &move_robot, py::arg("index"), py::arg("xShift"), py::arg("yShift"), py::arg("zShift"), "Moves the robot at the given index by specified shift values in the x, y, and z directions");
    m.def("move_robots_apart", &move_robots_apart, py::arg("distance"), "Moves robots apart so they dont spawn on the same place");

    m.def("simulate_step", &simulate_step, "Simulate one step");
    m.def("simulate_steps", &simulate_steps, py::arg("n"), "Simulate n steps");

    m.attr("ROLL_DAMPING") = &ROLL_DAMPING;

    // Global classes. definition is required to make robot attributes readable
    py::class_<_triMesh>(m, "TriMesh")
        .def(py::init<>())
        .def_readwrite("pointNum", &_triMesh::pointNum)
        .def_readwrite("color", &_triMesh::color);

    py::class_<massPointProp>(m, "MassPointProp")
        .def(py::init<>())
        .def_readwrite("x", &massPointProp::x)
        .def_readwrite("y", &massPointProp::y)
        .def_readwrite("z", &massPointProp::z);


    // RigidBody class binding
    py::class_<RigidBody, std::shared_ptr<RigidBody>>(m, "RigidBody")
        .def(py::init<>())

         // basic attributes
        .def_readwrite("MeshCounter", &RigidBody::MeshCounter, "Number of meshes associated with this rigid body")
        .def_readwrite("bodyIDcounter", &RigidBody::bodyIDcounter, "Number of bodies")
        .def_readwrite("jointIDcounter", &RigidBody::jointIDcounter, "Number of joints")
        .def_readwrite("connectionIDcounter", &RigidBody::connectionIDcounter, "Number of connections")
        .def_readwrite("CMmass", &RigidBody::CMmass, "CMmass")
        .def_readwrite("alpha", &RigidBody::alpha, "alpha")
        .def_readwrite("friction", &RigidBody::friction, "Friction")
        .def_readwrite("bounce", &RigidBody::bounce, "bounce")
        .def_readwrite("contactHeight", &RigidBody::contactHeight, "contactHeight")
        .def_readwrite("ANGLE_STEP", &RigidBody::ANGLE_STEP, "ANGLE_STEP")

        // Methods for retrieving geometry and physical properties stored in arrays
        .def("get_triMesh", [](RigidBody &rb) {
            return std::vector<_triMesh>(rb.triMesh, rb.triMesh + rb.MeshCounter);
        })
        .def("get_massPoints", [](RigidBody &rb) {
            return std::vector<massPointProp>(rb.massPoint, rb.massPoint + MAX_POINTS);
        })
        .def("get_center_of_mass", [](RigidBody &rb) {
            return std::vector<double>(rb.CM, rb.CM + 3);
        })
        .def("get_joint_angles", [](RigidBody &rb) {
            return std::vector<double>(rb.angle, rb.angle + rb.jointIDcounter);
        });


    // Robot class binding (needed for CustomRobot)
    py::class_<Robot, std::shared_ptr<Robot>, RigidBody>(m, "Robot")
    .def(py::init<>());


    // CustomRobot bindings for robot creation
    py::class_<CustomRobot, std::shared_ptr<CustomRobot>, Robot>(m, "CustomRobot", py::keep_alive<1, 2>())
        .def(py::init<>())
        .def("set_simplify_mode", &CustomRobot::set_simplify_mode)
        .def("create_box", &CustomRobot::create_box)
        .def("create_cylinder", [](CustomRobot &self, float startX, float startY, float startZ,
                           float endX, float endY, float endZ,
                           float radius, float mass,
                           float colorR, float colorG, float colorB) {
            return self.create_cylinder(startX, startY, startZ, endX, endY, endZ, radius, mass, colorR, colorG, colorB);
        })
        .def("create_joint", &CustomRobot::create_joint)
        .def("create_point", &CustomRobot::create_point)
        .def("create_sphere", &CustomRobot::create_sphere)
        .def("create_ray", &CustomRobot::create_ray)
        .def("getMaxBodyID", &CustomRobot::getMaxBodyID)
        .def("calculate_center_of_mass", &CustomRobot::calculateCenterOfMass)
        .def("calculate_momentum_of_inertia", &CustomRobot::calculateMomentumOfInertia)
        .def("set_body_part_color", &RigidBody::changeObjectColor, py::arg("body_id"), py::arg("red"), py::arg("green"), py::arg("blue"))
        .def("finalize_construction", &CustomRobot::finalize_construction)
        .def("set_joint_angles", [](CustomRobot &self, const std::vector<float> &angles) {
            self.set_joint_angles(angles);
        })
        .def("move", &CustomRobot::move);
}
