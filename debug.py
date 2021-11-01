import numpy as np
import pygame
import hybrid_astar


def pixel2meter(pixel):
    # 1 pixel = 1 cm
    return float(pixel * 0.005)


def meter2pixel(meter):
    return int(np.rint(meter * 200.0))


class Debug:
    def __init__(self):
        # =======================
        obj_width_m = 0.5
        robot_width_m = 0.6
        grid_width_m = 0.15
        # =======================

        pygame.init()
        pygame.display.set_caption("Hybrid A* Path for Pushing")
        self.canvas_size = (1000, 700)
        self.canvas = pygame.display.set_mode(self.canvas_size)

        self.obstacle_list = []  # (pixel_xy, pixel_radius) pairs
        # self.obj_start = {"xy": (235, 184), "radian": np.radians(40)}
        # self.obj_goal = {"xy": (705, 484), "radian": np.radians(22)}
        self.obj_start = {"xy": (270, 325), "radian": np.radians(0)}
        self.obj_goal = {"xy": (400, 310), "radian": np.radians(0)}
        self.obj_width_pixel = meter2pixel(obj_width_m)
        self.robot_width_pixel = meter2pixel(robot_width_m)
        self.obj_radius_pixel = self.obj_width_pixel / np.sqrt(2)
        self.robot_radius_pixel = self.robot_width_pixel / np.sqrt(2)

        self.obj_xyt_path = []
        self.robot_xyt_path = []

        # command
        self.pause = False
        self.help_msg = "\n1: obstacle,\n2: object start,\n3: object goal,\n4: compute path (ori toggle / click: plan)\n5: check closest point\n"
        self.mode = 1
        self.obj_start_radian_setting = True
        self.obj_goal_radian_setting = True
        self.ignore_goal_orientation = False

        # algorithm

        self.grid_dxy_pixel = meter2pixel(grid_width_m)
        self.grid_dtheta_degree = 12
        self.astar = hybrid_astar.HybridAstar(
            pixel2meter(self.grid_dxy_pixel), np.radians(self.grid_dtheta_degree)
        )
        self.astar.set_successors()
        self.astar.set_collision_model(
            pixel2meter(self.obj_radius_pixel),
            pixel2meter(self.robot_radius_pixel),
            pixel2meter(self.obj_width_pixel),
            pixel2meter(self.robot_width_pixel),
        )

    def radian_round_for_grid(self, rad):
        deg = np.degrees(rad)
        index = int(np.rint((deg % 360) / self.grid_dtheta_degree))
        return np.radians(index * self.grid_dtheta_degree)

    def main(self):
        self.clear()
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if not self.pause:
                        self.event_mouse()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_BACKQUOTE:
                        self.pause = False
                    if not self.pause:
                        self.event_keyboard(event)
            if not self.pause:
                self.canvas.fill((255, 255, 255))
                self.draw()

    def event_mouse(self):
        pos = pygame.mouse.get_pos()
        # print(pos)
        if self.mode == 1:
            self.obstacle_list.append((pos, 3))
        elif self.mode == 2:
            if self.obj_start_radian_setting:
                self.obj_start["xy"] = pos
            else:
                dx = pos[0] - self.obj_start["xy"][0]
                dy = pos[1] - self.obj_start["xy"][1]
                self.obj_start["radian"] = np.arctan2(dy, dx)
            self.obj_start_radian_setting = not self.obj_start_radian_setting
            print("new pose, rad:{}".format(self.obj_start))
        elif self.mode == 3:
            if self.obj_goal_radian_setting:
                self.obj_goal["xy"] = pos
            else:
                dx = pos[0] - self.obj_goal["xy"][0]
                dy = pos[1] - self.obj_goal["xy"][1]
                self.obj_goal["radian"] = np.arctan2(dy, dx)
            self.obj_goal_radian_setting = not self.obj_goal_radian_setting
            print("new pose, rad:{}".format(self.obj_goal))
        elif self.mode == 4:
            self.set_obstacle_to_hybrid_aster()
            o_start = (
                pixel2meter(self.obj_start["xy"][0]),
                pixel2meter(self.obj_start["xy"][1]),
                self.obj_start["radian"],
            )
            o_goal = (
                pixel2meter(self.obj_goal["xy"][0]),
                pixel2meter(self.obj_goal["xy"][1]),
                self.obj_goal["radian"],
            )
            result = self.astar.compute_path(
                o_start, o_goal, self.ignore_goal_orientation
            )
            # print(result)
            print("job done!")
            self.obj_xyt_path = result[1]
            self.robot_xyt_path = result[2]

    def event_keyboard(self, event):
        if event.key == pygame.K_1:
            self.mode = 1
        elif event.key == pygame.K_2:
            self.mode = 2
        elif event.key == pygame.K_3:
            self.mode = 3
        elif event.key == pygame.K_4:
            if self.mode != 4:
                self.mode = 4
            else:
                self.ignore_goal_orientation = not self.ignore_goal_orientation
        elif event.key == pygame.K_5:
            self.mode = 5
            self.set_obstacle_to_hybrid_aster()
            query_pos = pygame.mouse.get_pos()
            query_meter = (pixel2meter(query_pos[0]), pixel2meter(query_pos[1]))
            closest_meter = self.astar.get_closest_point(query_meter)
            closest_pos = (meter2pixel(closest_meter[0]), meter2pixel(closest_meter[1]))
            print("-----")
            print("- query_pos:   {}".format(query_pos))
            # print("- closest_pos: {}".format(closest_pos))
            pygame.draw.circle(self.canvas, (0, 0, 255), query_pos, 5, 1)
            pygame.draw.circle(self.canvas, (255, 0, 0), closest_pos, 5, 1)
            # grid check
            index = self.astar.grid.find_grid_index((query_meter[0], query_meter[1], 0))
            xyt = self.astar.grid.find_real_xyt(index)
            index_pixel = (meter2pixel(xyt[0]), meter2pixel(xyt[1]))
            print("- index_pixel: {}".format(index_pixel))
            pygame.draw.circle(self.canvas, (200, 200, 0), index_pixel, 5, 1)
            self.draw_grid()
            self.set_pause()
        elif event.key == pygame.K_SPACE:
            self.clear()
        print("Mode: %d (ig: %s)" % (self.mode, self.ignore_goal_orientation))

    def set_obstacle_to_hybrid_aster(self):
        metric_points = [
            (pixel2meter(p[0][0]), pixel2meter(p[0][1])) for p in self.obstacle_list
        ]
        self.astar.set_obstacles(metric_points)

    def set_pause(self):
        self.pause = True
        print("PAUSED...(Press ` to release.)")

    def draw_grid(self):
        half = self.grid_dxy_pixel / 2
        max_x, max_y = self.canvas_size
        color = pygame.Color(200, 200, 200)
        width = 1
        # vertical
        x = half
        sy, ey = (0, max_y)
        while x < max_x:
            pygame.draw.line(self.canvas, color, (x, sy), (x, ey), width)
            x += self.grid_dxy_pixel
        # horizontal
        y = half
        sx, ex = (0, max_x)
        while y < max_y:
            pygame.draw.line(self.canvas, color, (sx, y), (ex, y), width)
            y += self.grid_dxy_pixel
        # directions

        def calc_dxy(rad):
            pixel_length = 20
            return (np.cos(rad) * pixel_length, np.sin(rad) * pixel_length)

        start_xy = self.obj_start["xy"]
        for deg in range(0, 360, self.grid_dtheta_degree):
            dx, dy = calc_dxy(np.radians(deg))
            end = (start_xy[0] + dx, start_xy[1] + dy)
            pygame.draw.line(self.canvas, (0, 0, 0), start_xy, end, 1)

        # successor
        start_xy_meter = (pixel2meter(start_xy[0]), pixel2meter(start_xy[1]))
        start_rad = self.radian_round_for_grid(self.obj_start["radian"])
        successors = self.astar.get_successors(start_xy_meter, start_rad)
        # print(successors)
        color = (255, 0, 0)
        for stype, s in successors:
            xy, rad, cost, icp_xy = s
            x_meter, y_meter = xy
            xy_pixel = (meter2pixel(x_meter), meter2pixel(y_meter))
            pygame.draw.circle(self.canvas, color, xy_pixel, 3, 0)
            # vector
            dx, dy = calc_dxy(rad)
            end = (xy_pixel[0] + dx, xy_pixel[1] + dy)
            pygame.draw.line(self.canvas, color, xy_pixel, end, 2)
        self.draw()

    def draw(self):
        # obstacles
        for pixel_xy, pixel_radius in self.obstacle_list:
            pygame.draw.circle(self.canvas, (0, 0, 0), pixel_xy, pixel_radius, 0)
        # object - start, goal
        self.draw_object(self.obj_start["xy"], self.obj_start["radian"], 255, 2)
        self.draw_object(self.obj_goal["xy"], self.obj_goal["radian"], 255, 4)
        # paths
        for xyt in self.obj_xyt_path:
            x, y, rad = xyt
            xy_pixel = (meter2pixel(x), meter2pixel(y))
            self.draw_object(xy_pixel, rad, 80, 2, remove_circle=True)
        for xyt in self.robot_xyt_path:
            x, y, rad = xyt
            xy_pixel = (meter2pixel(x), meter2pixel(y))
            self.draw_robot(xy_pixel, rad, 20, 2, remove_circle=True)
        pygame.display.update()

    def draw_robot(self, xy_pixel, radian, alpha, line_width, remove_circle=False, remove_square=False):
        color = pygame.Color(0, 0, 255, alpha)
        self.draw_circle_collision(
            xy_pixel, radian, self.robot_radius_pixel, color, line_width, remove_circle, remove_square
        )

    def draw_object(self, xy_pixel, radian, alpha, line_width, remove_circle=False, remove_square=False):
        color = pygame.Color(0, 220, 0, alpha)
        self.draw_circle_collision(
            xy_pixel, radian, self.obj_radius_pixel, color, line_width, remove_circle, remove_square
        )

    def draw_circle_collision(
        self, xy_pixel, radian, radius, color, line_width, remove_circle, remove_square
    ):
        surface = pygame.Surface(self.canvas_size, pygame.SRCALPHA)
        if not remove_circle:
            pygame.draw.circle(surface, color, xy_pixel, radius, line_width)
        # heading
        x = xy_pixel[0] + (radius * np.cos(radian))
        y = xy_pixel[1] + (radius * np.sin(radian))
        pygame.draw.line(surface, color, xy_pixel, (x, y), line_width)
        # square
        if not remove_square:
            quarter = np.pi / 4.0
            rads = [quarter, quarter * 3.0, -quarter * 3.0, -quarter]
            points = [
                (
                    xy_pixel[0] + (radius * np.cos(radian + q)),
                    xy_pixel[1] + (radius * np.sin(radian + q)),
                )
                for q in rads
            ]
            pygame.draw.lines(surface, color, True, points, line_width)
        self.canvas.blit(surface, (0, 0))

    def clear(self):
        self.obstacle_list = []
        print(self.help_msg)


if __name__ == "__main__":
    d = Debug()
    d.main()
