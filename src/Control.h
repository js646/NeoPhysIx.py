#include "GlobalDefs.h"
#include "Environment.h"
#include "Robot.h"
#include "robots/CustomRobot.h"

#include <vector>
#include <memory> // adds std::shared_ptr definitions

// extern declarations
extern environment world;
extern double landscape[MAX_WORLD+1][MAX_WORLD+1];
extern std::vector<std::shared_ptr<Robot>> robots;
extern float ROLL_DAMPING;

// provided methods (API)
void set_environment(EnvironmentType env_type);
void set_gravity(double value);

void add_robots(RobotType robot_type, int robot_count);
void add_custom_robot(std::shared_ptr<CustomRobot> custom_robot);

void move_robot(int index, double xShift, double yShift, double zShift);
void move_robots_apart(int distance);

void simulate_step();
void simulate_steps(int n);

// helper methods (used internally)
std::vector<Robot*> getRawPointers(const std::vector<std::shared_ptr<Robot>>& robot_shared_ptrs);
