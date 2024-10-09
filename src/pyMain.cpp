#include <memory>
#include <vector>
#include <stdexcept>

#include "pyMain.h"
#include "GlobalDefs.h"
#include "Math.h"

#include "Environment.h"

#include "Robot.h"
#include "robots/FourLeggedRobot.h"
#include "robots/FourLeggedComparison.h"
#include "robots/Humanoid.h"
#include "robots/SixLeggedRobot.h"
#include "robots/WheeledRobot.h"
#include "robots/CustomRobot.h"

#include "PhysicsEngine.h"

// environment
environment world;
double landscape[MAX_WORLD+1][MAX_WORLD+1];
EnvironmentType current_environment = EnvironmentType::Plane; // default == plane

// robots
std::vector<std::unique_ptr<Robot>> robots;

// physics
physics physics_engine;

// ------ API METHODS -----

// Used to set the environment type
void set_environment(EnvironmentType env_type) {
    current_environment = env_type;
}

// Used to set the robot type and amount of robots
void set_robots(RobotType robot_type, int robot_count) {
    robots.clear();

    for (int i = 0; i < robot_count; i++) {
        switch (robot_type) {
            case RobotType::FourLegged:
                robots.push_back(std::make_unique<FourLeggedRobot>());
                break;
            case RobotType::FourLeggedComparison:
                robots.push_back(std::make_unique<FourLeggedComparison>());
                break;
            case RobotType::SixLegged:
                robots.push_back(std::make_unique<SixLeggedRobot>());
                break;
            case RobotType::Humanoid:
                robots.push_back(std::make_unique<Humanoid>());
                #define ROLL_DAMPING 0.99f
                break;
            case RobotType::Wheeled:
                robots.push_back(std::make_unique<WheeledRobot>());
                break;
            case RobotType::Custom:
                robots.push_back(std::make_unique<CustomRobot>());
                break;
            default:
                throw std::runtime_error("Unknown robot type");
        }
    }

    if(robot_count > 1){
        move_robots_apart(1);
    }
}

// Initializes the simulation (has to be called AFTER set_environment)
void init_simulation(){
    // Initialize environment (initializes the landscape array)
    world = environment();
}

// simulates n steps
void simulate_steps(int n){
    for (int i = 0; i < n; i++) {
        for (auto& robot : robots) {
            physics_engine.simulateStep(robot.get(), 1);
        }
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