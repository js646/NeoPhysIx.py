import pygame
import time

from graphics import Graphics

try:
    import NeoPhysix
    print("NeoPhysix imported successfully!")
except ImportError as e:
    print(f"Error while importing NeoPhysix: {e}")
except Exception as e:
    print(f"An error occurred: {e}")


class App:
    def __init__(self):
        pygame.init()  # init pygame
        self.graphics = Graphics()  # init graphics

        # set the environment type (Plane, Ramp, Mountain, Apartment)
        NeoPhysix.set_environment(NeoPhysix.EnvironmentType.Ramp)

        # set the robot type (FourLegged, FourLeggedComparison, SixLegged, Humanoid, Wheeled or Custom) and amount
        NeoPhysix.set_robots(NeoPhysix.RobotType.FourLegged, 1)

        # init simulation
        NeoPhysix.init_simulation()

        # get landscape data from NeoPhysix
        self.landscape = NeoPhysix.get_landscape()

        # get robot data from NeoPhysix
        self.robots = NeoPhysix.get_robots()

        # simulation variables & counters
        self.GFX_STEP = 100  # graphics gets painted all GFX_STEP steps
        self.step_counter = 0  # total amount of simulated steps

        # enter main loop
        self.main_loop()

    def main_loop(self):
        running = True
        run_time_start = time.perf_counter()
        while running:
            # check and handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                else:
                    # handle key controls
                    self.graphics.handle_control_events(event)
                    # handle mouse interactions
                    self.graphics.handle_mouse_actions()

            # capture start time
            cpu_time_start = time.perf_counter()

            # simulate steps
            NeoPhysix.simulate_steps(self.GFX_STEP)

            # capture end time
            cpu_time_end = time.perf_counter()

            # calc cpu time for simulated steps
            cpu_time = cpu_time_end - cpu_time_start

            # calc runtime
            run_time = time.perf_counter() - run_time_start

            # count simulated steps
            self.step_counter += self.GFX_STEP

            # draw current scene and text
            self.graphics.draw_scene(self.landscape, self.robots, 128)
            self.graphics.draw_text(cpu_time, self.GFX_STEP, self.step_counter, run_time, (10, 10))
            self.graphics.update_screen()

            # print joint angles
            for robot in self.robots:
                print(str(robot.get_joint_angles()))

        self.quit()

    def quit(self):
        self.graphics.destroy()
        pygame.quit()


if __name__ == '__main__':
    myApp = App()
