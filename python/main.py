import pygame
import time

from graphics import Graphics

try:
    import NeoPhysIx

    print("NeoPhysIx imported successfully!")
except ImportError as e:
    print(f"Error while importing NeoPhysIx: {e}")
except Exception as e:
    print(f"An error occurred: {e}")


def create_quadruped_robot():
    robot = NeoPhysIx.CustomRobot()

    robot.set_simplify_mode(0)

    SCALE = 0.6
    radius = 0.08 * SCALE
    idUpper = [0, 0, 0, 0, 0, 0]
    idLower = [0, 0, 0, 0, 0, 0]

    idBox = robot.create_box(0.2 * SCALE, 0.2 * SCALE, 0.2 * SCALE, 0, 0, 0, 0.8, 0.7, 0.3, 0.3)
    idUpper[0] = robot.create_cylinder(0.1 * SCALE, 0 * SCALE, 0 * SCALE, 0.4 * SCALE, 0.4 * SCALE,
                                       0 * SCALE, radius, 0.2, 0.3, 0.3, 0.7)
    idLower[0] = robot.create_cylinder(0.4 * SCALE, 0.4 * SCALE, 0 * SCALE, 0.9 * SCALE, -0.7 * SCALE,
                                       0 * SCALE, radius, 0.2, 0.3, 0.7, 0.3)
    idUpper[1] = robot.create_cylinder(-0.1 * SCALE, 0 * SCALE, 0 * SCALE, -0.4 * SCALE, 0.4 * SCALE,
                                       0 * SCALE, radius, 0.2, 0.3, 0.3, 0.7)
    idLower[1] = robot.create_cylinder(-0.4 * SCALE, 0.4 * SCALE, 0 * SCALE, -0.9 * SCALE, -0.7 * SCALE,
                                       0 * SCALE, radius, 0.2, 0.3, 0.7, 0.3)
    idUpper[2] = robot.create_cylinder(0 * SCALE, 0 * SCALE, 0.1 * SCALE, 0 * SCALE, 0.4 * SCALE,
                                       0.4 * SCALE, radius, 0.2, 0.3, 0.3, 0.7)
    idLower[2] = robot.create_cylinder(0 * SCALE, 0.4 * SCALE, 0.4 * SCALE, 0 * SCALE, -0.7 * SCALE,
                                       0.9 * SCALE, radius, 0.2, 0.3, 0.7, 0.3)
    idUpper[3] = robot.create_cylinder(0 * SCALE, 0 * SCALE, -0.1 * SCALE, 0 * SCALE, 0.4 * SCALE,
                                       -0.4 * SCALE, radius, 0.2, 0.3, 0.3, 0.7)
    idLower[3] = robot.create_cylinder(0 * SCALE, 0.4 * SCALE, -0.4 * SCALE, 0 * SCALE, -0.7 * SCALE,
                                       -0.9 * SCALE, radius, 0.2, 0.3, 0.7, 0.3)

    robot.create_joint(idBox, idUpper[0], 0.1 * SCALE, 0, 0, 0, 0, 1)
    robot.create_joint(idUpper[0], idLower[0], 0.4 * SCALE, 0.4 * SCALE, 0, 0, 0, 1)

    robot.create_joint(idBox, idUpper[1], -0.1 * SCALE, 0, 0, 0, 0, 1)
    robot.create_joint(idUpper[1], idLower[1], -0.4 * SCALE, 0.4 * SCALE, 0, 0, 0, 1)

    robot.create_joint(idBox, idUpper[2], 0, 0, 0.1 * SCALE, 1, 0, 0)
    robot.create_joint(idUpper[2], idLower[2], 1, 0.4 * SCALE, 0.4 * SCALE, 1, 0, 0)

    robot.create_joint(idBox, idUpper[3], 0, 0, -0.1 * SCALE, 1, 0, 0)
    robot.create_joint(idUpper[3], idLower[3], 0, 0.4 * SCALE, -0.4 * SCALE, 1, 0, 0)

    robot.finalize_construction()

    return robot


class App:
    def __init__(self):
        pygame.init()  # init pygame

        self.graphics = Graphics()  # init graphics

        # set the environment type (Plane, Ramp, Mountain, Apartment)
        NeoPhysIx.set_environment(NeoPhysIx.EnvironmentType.Plane)

        # set the robot type and amount (FourLegged, FourLeggedComparison, SixLegged, Humanoid or Wheeled)
        NeoPhysIx.add_robots(NeoPhysIx.RobotType.FourLegged, 1)

        # create and add custom robot
        self.custom_robot = create_quadruped_robot()
        NeoPhysIx.add_custom_robot(self.custom_robot)

        # move robots apart, so they don't spawn on the same place
        NeoPhysIx.move_robots_apart(1)
        # NeoPhysIx.move_robot(0, -2, 0, -2)

        # get robot and landscape data from NeoPhysix
        self.landscape = NeoPhysIx.get_landscape()
        self.robots = NeoPhysIx.get_robots()

        # simulation variables & counters
        self.GFX_STEP = 100  # graphics gets painted after GFX_STEP simulation steps
        self.step_counter = 0  # total amount of simulated steps

        # enter main loop
        self.main_loop()

    def main_loop(self):
        running = True
        run_time_start = time.perf_counter()
        while running:
            # check and handle input events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                else:
                    # handle key controls
                    self.graphics.handle_control_events(event)
                    # handle mouse interactions
                    self.graphics.handle_mouse_actions()

            # influence CustomRobot movements
            # self.robots[1].set_joint_angles([0, 0.2, 0.5, 0])

            # capture start time
            cpu_time_start = time.perf_counter()

            # simulate steps
            NeoPhysIx.simulate_steps(self.GFX_STEP)

            # time measurements
            cpu_time_end = time.perf_counter()  # capture end time
            cpu_time = cpu_time_end - cpu_time_start  # calc cpu
            run_time = time.perf_counter() - run_time_start  # calc runtime
            self.step_counter += self.GFX_STEP  # count simulated steps

            # draw current scene and text
            self.graphics.draw_scene(self.landscape, self.robots, 128)
            self.graphics.draw_text(cpu_time, run_time, self.GFX_STEP, self.step_counter, 10, 10)
            self.graphics.update_screen()

            # self.print_robot_data()
        self.quit()

    def print_robot_data(self):
        for robot in self.robots:
            print("joint angles: " + str(robot.get_joint_angles()))
            print("triMesh pointNum: " + str(robot.get_triMesh()[1].pointNum))
            print("massPoints[0]: " + str(robot.get_massPoints()[0].x) + " " + str(robot.get_massPoints()[0].y)
                  + " " + str(robot.get_massPoints()[0].z))
            print("center_of_mass: " + str(robot.get_center_of_mass()[0]) + " " + str(robot.get_center_of_mass()[1])
                  + " " + str(robot.get_center_of_mass()[2]))
            print("MeshCounter: " + str(robot.MeshCounter))
            print("bodyIDcounter: " + str(robot.bodyIDcounter))
            print("jointIDcounter: " + str(robot.jointIDcounter))
            print("connectionIDcounter: " + str(robot.connectionIDcounter))
            print("CMmass: " + str(robot.CMmass))
            print("alpha: " + str(robot.alpha))
            print("friction: " + str(robot.friction))
            print("bounce: " + str(robot.bounce))
            print("contactHeight: " + str(robot.contactHeight))
            print("ANGLE_STEP: " + str(robot.ANGLE_STEP))

    def quit(self):
        self.graphics.destroy()
        pygame.quit()


if __name__ == '__main__':
    myApp = App()
