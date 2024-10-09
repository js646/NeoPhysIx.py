#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h> // Für die Konvertierung von std::vector nach Python-Liste

#include <array>
#include <vector>
#include <memory>

#include "src/GlobalDefs.h"
#include "src/RigidBody.h"
#include "src/Robot.h"
#include "src/robots/FourLeggedRobot.h"
#include "src/robots/FourLeggedComparison.h"
#include "src/robots/Humanoid.h"
#include "src/robots/SixLeggedRobot.h"
#include "src/robots/WheeledRobot.h"
#include "src/pyMain.h"

namespace py = pybind11;

// Getter for the landscape array
py::array_t<double> get_landscape() {
    return py::array_t<double>(
        {MAX_WORLD+1, MAX_WORLD+1}, // shape
        {sizeof(double) * (MAX_WORLD+1), sizeof(double)}, // strides
        &landscape[0][0] // data pointer
    );
}

// Getter for robots array
std::vector<Robot*> get_robots() {
    std::vector<Robot*> robot_list;
    for (const auto& robot : robots) {
        robot_list.push_back(robot.get());
    }
    return robot_list;
}

PYBIND11_MODULE(NeoPhysix, m) {
    m.doc() = "NeoPhysix binding module";

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
        .value("Wheeled", RobotType::Wheeled)
        .value("Custom", RobotType::Custom);


    // Methods to interact with NeoPhysIx
    m.def("set_robots", &set_robots, py::arg("robot_type"), py::arg("robot_count"), "Set the robots in the simulation");
    m.def("set_environment", &set_environment, py::arg("env_type"), "Set the environment for the simulation");
    m.def("init_simulation", &init_simulation, "Initialize the simulation");
    m.def("simulate_steps", &simulate_steps, py::arg("n"), "Simulate n steps");


    // Getter methods provided by this module
    m.def("get_landscape", &get_landscape, "Get the landscape array");
    m.def("get_robots", &get_robots, "Get the list of all robots");


    // Global classes. definition is required to be able to read robot attributes
    py::class_<_triMesh>(m, "TriMesh")
        .def(py::init<>())
        .def_readwrite("pointNum", &_triMesh::pointNum)
        .def_readwrite("color", &_triMesh::color);

    py::class_<massPointProp>(m, "MassPointProp")
        .def(py::init<>())
        .def_readwrite("x", &massPointProp::x)
        .def_readwrite("y", &massPointProp::y)
        .def_readwrite("z", &massPointProp::z);


    // RigidBody / Robot class definitions. required to be able to read robot attributes
    py::class_<RigidBody>(m, "RigidBody")
        .def(py::init<>())
        .def_readwrite("MeshCounter", &RigidBody::MeshCounter)
        .def("get_triMesh", [](RigidBody &rb) {
            return std::vector<_triMesh>(rb.triMesh, rb.triMesh + rb.MeshCounter);
        })
        .def("get_massPoints", [](RigidBody &rb) {
            return std::vector<massPointProp>(rb.massPoint, rb.massPoint + MAX_POINTS);
        })
        .def("get_joint_angles", [](RigidBody &rb) {
            return std::vector<double>(rb.angle, rb.angle + rb.jointIDcounter);
        });

    py::class_<Robot, RigidBody>(m, "Robot")
        .def(py::init<>());
}
