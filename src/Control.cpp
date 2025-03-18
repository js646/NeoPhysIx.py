#include <memory>
#include <vector>
#include <stdexcept>

#include <iostream>

#include "Control.h"
#include "GlobalDefs.h"
#include "Math.h"

#include "Environment.h"

#include "Robot.h"
#include "robots/FourLeggedRobot.h"
#include "robots/FourLeggedComparison.h"
#include "robots/Humanoid.h"
#include "robots/SixLeggedRobot.h"
#include "robots/WheeledRobot.h"

#include "PhysicsEngine.h"

// environment
environment world;
double landscape[MAX_WORLD+1][MAX_WORLD+1];
EnvironmentType current_environment = EnvironmentType::Plane; // default == plane

// robots
std::vector<std::shared_ptr<Robot>> robots;

// physics engine
physics physics_engine;

// simulation parameters
float ROLL_DAMPING = 0.8f;

// ------ API METHODS -----

// Used to set the environment type
void set_environment(EnvironmentType env_type) {
    current_environment = env_type;
    world = environment();
}

// Used to set the gravity value used in the physics engine (default: -9.81)
void set_gravity(double value){
    physics_engine.setGravity(value);
}

// Used add predefined robots to the simulation
void add_robots(RobotType robot_type, int robot_count) {
    for (int i = 0; i < robot_count; i++) {
        switch (robot_type) {
            case RobotType::FourLegged:
                robots.push_back(std::make_shared<FourLeggedRobot>());
                break;
            case RobotType::FourLeggedComparison:
                robots.push_back(std::make_shared<FourLeggedComparison>());
                break;
            case RobotType::SixLegged:
                robots.push_back(std::make_shared<SixLeggedRobot>());
                break;
            case RobotType::Humanoid:
                robots.push_back(std::make_shared<Humanoid>());
                ROLL_DAMPING = 0.99f;
                break;
            case RobotType::Wheeled:
                robots.push_back(std::make_shared<WheeledRobot>());
                break;
            default:
                throw std::runtime_error("Unknown robot type");
        }
    }
}

// Used to add a custom robot to the simulation
void add_custom_robot(std::shared_ptr<CustomRobot> custom_robot) {
    robots.push_back(custom_robot);
}

// Clears the robots array
void clear_robots(){
    robots.clear();
}

// simulates one single step
void simulate_step() {
    auto robot_raw_ptrs = getRawPointers(robots);
    physics_engine.simulateStep(robot_raw_ptrs);
}

// simulates n steps
void simulate_steps(int n) {
    auto robot_raw_ptrs = getRawPointers(robots);
    for (int i = 0; i < n; i++) {
        physics_engine.simulateStep(robot_raw_ptrs);
    }
}

// moves the robot at the given index by specified shift values in the x, y, and z directions
void move_robot(int index, double xShift, double yShift, double zShift){
    int i = 0;
    for (auto& robot : robots) {
        if(i == index){
            for (int bodyID = 0; bodyID < robot->bodyIDcounter; bodyID++) {
                robot->moveBody(bodyID, xShift, yShift, zShift);
            }
            break;  // Exit loop after moving the target robot
        }
        i++;
    }
}

// moves robots apart so they dont spawn on the same place
void move_robots_apart(int distance){
    double shift = 0;
    for (auto& robot : robots) {
        shift += distance; // Move each robot "distance" units apart in the z-axis
        for (int bodyID = 0; bodyID < robot->bodyIDcounter; bodyID++) {
            robot->moveBody(bodyID, 1, 0, shift); // adjust coordinates as needed
        }
    }
}

// ------ HELPER METHODS -----

// Helper to convert shared_ptr list to raw pointers for simulation
std::vector<Robot*> getRawPointers(const std::vector<std::shared_ptr<Robot>>& robot_shared_ptrs) {
    std::vector<Robot*> robot_raw_ptrs;
    robot_raw_ptrs.reserve(robot_shared_ptrs.size());
    for (const auto& robot : robot_shared_ptrs) {
        robot_raw_ptrs.push_back(robot.get());
    }
    return robot_raw_ptrs;
}