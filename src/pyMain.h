#ifndef PY_MAIN_H
#define PY_MAIN_H

#include "GlobalDefs.h"
#include "Environment.h"
#include "Robot.h"

#include <vector>
#include <memory> // Fügt die std::unique_ptr Definition hinzu

// extern declarations
extern environment world;
extern double landscape[MAX_WORLD+1][MAX_WORLD+1];
extern std::vector<std::unique_ptr<Robot>> robots;

// provided methods (API)
void set_environment(EnvironmentType env_type);
void set_robots(RobotType robot_type, int robot_count);
void init_simulation();
void simulate_steps(int n);

// internally used methods
void move_robots_apart(int distance);


#endif // PY_MAIN_H
