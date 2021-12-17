import heapq
import time
import numpy as np
from scipy.spatial import cKDTree


class Collision:
    def __init__(self, radius, width):
        self.radius = radius
        self.width = width


class Node:
    def __init__(self, real_xyt, grid_index):
        """
        real_xyt: m, m, rad
        """
        self._real_xyt = real_xyt
        self._grid_index = grid_index

        # came from
        self.parent_index = None
        self.steering_type = None  # +:ccw, -:cw

        self._g_score = np.inf
        self._h_score = 0.0
        self._f_score = self._g_score + self._h_score

    @property
    def xyt(self):
        return self._real_xyt

    @property
    def xy(self):
        return (self._real_xyt[0], self._real_xyt[1])

    @property
    def rad(self):
        return self._real_xyt[2]

    @property
    def index(self):
        return self._grid_index

    @property
    def g(self):
        return self._g_score

    @property
    def h(self):
        return self._h_score

    @property
    def f(self):
        return self._f_score

    def set_xyt(self, xyt):
        self._real_xyt = xyt

    def set_score(self, g, h):
        self._g_score = g
        self._h_score = h
        self._f_score = self._g_score + self._h_score

    def __repr__(self):
        x, y, rad = self._real_xyt
        deg = np.degrees(rad)
        return "\n<Node({:.2f},{:.2f},{:.0f}) parent[{}] f{:.4f}=g{:.4f}+h{:.4f}>".format(x, y, deg, self.index, self.f, self.g, self.h)

    # # Customs for GridMap
    # def __hash__(self):
    #     return self._grid_xyt

    # Customs for OpenList
    # (<, <=, >, >=, == and !=) can be overloaded by providing definition
    #  to __lt__, __le__, __gt__, __ge__, __eq__ and __ne__ magic methods.
    def __lt__(self, other):
        return self.f < other.f


class Grid:
    def __init__(self, dxy_meter, dtheta_radian):
        self.dxy_meter_float = dxy_meter
        self.dtheta_deg_int = int(np.rint(np.degrees(dtheta_radian)))
        if 360 % self.dtheta_deg_int:
            raise ValueError("360 % np.degrees(dtheta) must be 0.")
        self._data = {}

    def find_grid_index(self, real_xyt):
        x_index = int(np.rint(real_xyt[0] / self.dxy_meter_float))
        y_index = int(np.rint(real_xyt[1] / self.dxy_meter_float))
        real_deg = np.degrees(real_xyt[2])
        t_index = int(np.rint(real_deg % 360) / self.dtheta_deg_int)
        return (x_index, y_index, t_index)

    def find_real_xyt(self, grid_index):
        x = grid_index[0] * self.dxy_meter_float
        y = grid_index[1] * self.dxy_meter_float
        t = np.radians(grid_index[2] * self.dtheta_deg_int)
        return (x, y, t)

    def get_node_from_index(self, index):
        return self._data[index]

    def get_node(self, xyt):
        index = self.find_grid_index(xyt)
        if index in self._data:
            return self._data[index]
        else:
            n = Node(xyt, index)
            self._data[index] = n
            return n


class HybridAstar:
    def __init__(self, dxy_meter, dtheta_radian, xmin, xmax, ymin, ymax):
        self.grid_xy_size = dxy_meter
        self.min_drad = dtheta_radian
        print("=== dxy_meter, drad = {:.2f}, {:.1f}".format(dxy_meter, dtheta_radian))
        self.grid = Grid(self.grid_xy_size, self.min_drad)
        self.workspace = (xmin, xmax, ymin, ymax)
        self.min_succesor_dist = dxy_meter * np.sqrt(2)

        # obstacle
        self.obstacles = None  # array
        self.kdtree = None
        # successor [(type, successor)]
        self.successors = []
        # collision
        self.object_collision = None
        self.robot_collision = None

        self.conditions = {
            "circle_obstacles": False,
            "successor": False,
            "collision_model": False,
        }

    def set_obstacles(self, point_list):
        self.obstacles = np.array(point_list if point_list else [(-100, -100)])
        # print(self.obstacles)
        self.kdtree = cKDTree(self.obstacles)
        self.conditions["circle_obstacles"] = True

    def get_closest_point(self, query_xy):
        distance, index = self.kdtree.query(query_xy)
        # print(distance)
        closest_xy = self.obstacles[index]
        return closest_xy

    def get_closest_distance(self, query_xy):
        distance, index = self.kdtree.query(query_xy)
        return distance

    def set_successors(self, stype="wedge"):
        diagonal_length = self.min_succesor_dist  # 0.15 * root(2) m
        min_drad = self.min_drad  # 12 deg

        self.successors = []
        if stype == "wedge":
            self.set_wedge_successors(diagonal_length, min_drad)
        elif stype == "full_circle":
            # self.set_circle_successors(diagonal_length, min_drad, np.radians(36.01))  # (mu: 4.2654, alpha: 76.806 deg)
            self.set_circle_successors(diagonal_length, min_drad, np.radians(80.))
        elif stype == "sm_circle":
            self.set_circle_successors(diagonal_length, min_drad, np.radians(12.01))  # (mu: 0.5648, alpha: 29.458 deg)
        else:
            raise ValueError("Incorrect Successor Type!")

        self.conditions["successor"] = True

    def set_circle_successors(self, _min_succesor_dist, _min_drad, max_heading_change):
        # chair width = 44 cm
        _min_succesor_dist  # 15*root(2) = 21.2 cm /2 = 10.6
        _min_drad  # 12 deg

        def calc_csucc(d_heading_rad):
            if d_heading_rad != 0.0:
                _r = _min_succesor_dist / 2.0
                dx = _r * (1.0 + np.cos(d_heading_rad))
                dy = _r * np.sin(d_heading_rad)
                dtheta = d_heading_rad
                icp_x = 0.0
                icp_y = _r / np.arctan(-d_heading_rad / 2.0)
                cost = abs(icp_y) * d_heading_rad
            else:
                dx = _min_succesor_dist
                dy = 0.0
                dtheta = 0.0
                cost = _min_succesor_dist
                icp_x = 0.0
                icp_y = np.inf
            return ((dx, dy), dtheta, cost, (icp_x, icp_y))

        # (dxy, dtheta, cost, icp_xy) [meter, rad]
        drad_list = [(0, 0)]  # type, rad
        # type(+), rad(+): ccw
        _rad = _min_drad
        stype = 1
        while _rad < max_heading_change:
            drad_list.append((stype, _rad))
            drad_list.append((-stype, -_rad))
            stype += 1
            _rad += _min_drad

        for style, rad in drad_list:
            dxy, dtheta, cost, icp_xy = calc_csucc(rad)
            fw = (style, (dxy, dtheta, cost, icp_xy))
            self.successors.append(fw)

    def set_wedge_successors(self, _min_succesor_dist, _min_drad):
        diagonal_length = _min_succesor_dist

        def calc_rotation_radius(d_heading_radian):
            radius = np.inf
            theta = abs(d_heading_radian)
            if d_heading_radian > 0.0:
                denominator = np.sqrt(2.0 * (1.0 - np.cos(theta)))
                radius = diagonal_length / denominator
            return radius

        def calc_successor(d_heading_radian):
            if d_heading_radian != 0.0:
                sign = np.sign(d_heading_radian)
                abs_theta = abs(d_heading_radian)
                r = calc_rotation_radius(abs_theta)

                dx = r * np.sin(abs_theta)
                dy = r * (1.0 - np.cos(abs_theta)) * sign
                dtheta = d_heading_radian
                cost = r * abs_theta
                icp_x = 0.0
                icp_y = r * sign
            else:
                dx = diagonal_length
                dy = 0.0
                dtheta = 0.0
                cost = diagonal_length
                icp_x = 0.0
                icp_y = np.inf
            return ((dx, dy), dtheta, cost, (icp_x, icp_y))

        # (dxy, dtheta, cost, icp_xy) [meter, rad]
        drad_list = [(0, 0)]  # type, rad
        # type(+), rad(+): ccw
        max_rad = np.radians(45)
        _rad = _min_drad
        stype = 1
        while _rad < max_rad:
            drad_list.append((stype, _rad))
            drad_list.append((-stype, -_rad))
            stype += 1
            _rad += _min_drad
        print("drad_list={}".format(drad_list))

        self.successors = [(stype, calc_successor(rad)) for stype, rad in drad_list]
        self.conditions["successor"] = True

    def get_successors(self, current_xy, current_rad):
        _c = np.cos(current_rad)
        _s = np.sin(current_rad)
        R = np.array([[_c, -_s], [_s, _c]])

        def transformation(local_xy):
            new_xy = np.dot(R, np.array(local_xy)) + np.array(current_xy)
            return new_xy

        def for_each(one_successor):
            dxy, dtheta, cost, icp_xy = one_successor
            new_xy = transformation(dxy)
            new_theta = current_rad + dtheta
            new_icp_xy = transformation(icp_xy)
            return (new_xy, new_theta, cost, new_icp_xy)

        successors = [(stype, for_each(s)) for stype, s in self.successors]
        return successors

    def set_collision_model(self, object_radius, robot_radius, object_width, robot_width):
        self.object_collision = Collision(object_radius, object_width)
        self.robot_collision = Collision(robot_radius, robot_width)
        self.conditions["collision_model"] = True

    def is_in_workspace(self, xy):
        x, y = xy
        xmin, xmax, ymin, ymax = self.workspace
        return (x < xmin) or (x > xmax) or (y < ymin) or (y > ymax)

    def check_collision(self, obj_xyt):
        obj_xy = (obj_xyt[0], obj_xyt[1])
        object_collide = self.get_closest_distance(obj_xy) < self.object_collision.radius
        robot_xyt = self.calc_robot_xyt(obj_xyt)
        rb_xy = (robot_xyt[0], robot_xyt[1])
        robot_collide = self.get_closest_distance(rb_xy) < self.robot_collision.radius
        out_of_workspace = (not self.is_in_workspace(obj_xy)) or (not self.is_in_workspace(rb_xy))
        return (object_collide or robot_collide)

    def compute_path(self, obj_start, obj_goal, ignore_goal_orientation):
        """
        obj_start: pose2d (x, y, heading) [m, m, rad]
        obj_goal: pose2d (x, y, heading) [m, m, rad]
        ignore_goal_orientation: bool
        ---
        path_length: int
        object_path: (non-holonomic)
        robot_path: (holonomic)
        ============
        1. ignore_goal_orientation option
        2. robot(holonomic) and object(non-holonomic) collision
        3. minimum steering heuristic
        """
        if False in self.conditions.values():
            raise ValueError("Some conditions are not prepared.")

        self.grid = Grid(self.grid_xy_size, self.min_drad)

        def heuristic(query_xyt):
            qx, qy, qrad = query_xyt
            gx, gy, grad = obj_goal
            dist = np.sqrt((qx - gx)**2 + (qy - gy)**2)
            if not ignore_goal_orientation:
                # IMPORTANT
                min_rotation_radius = 0.3
                drad = abs(qrad - grad)
                arc_length = min_rotation_radius * drad
                dist += arc_length
            return dist

        def steering_cost(prev_stype, successor_stype):
            if prev_stype is not None:
                dstype = abs(successor_stype - prev_stype)
                drad = dstype * self.min_drad
                pseudo_dy = drad * self.min_succesor_dist
                return pseudo_dy
            return 0.0

        # MAIN ALGORITHM
        start = self.grid.get_node(obj_start)
        start.set_score(0.0, heuristic(obj_start))
        open_heap = []
        heapq.heappush(open_heap, start)
        is_not_sorted = False

        goal_index = self.grid.find_grid_index(obj_goal)
        begin_time = time.time()
        timeout = 10
        print("computing...")
        while open_heap:
            elapsed = time.time() - begin_time
            if elapsed > timeout:
                break
            if is_not_sorted:
                heapq.heapify(open_heap)
                is_not_sorted = False
            current = heapq.heappop(open_heap)

            # print("\ncur:{}\top:{}".format(current, open_heap))

            if (current.index[0] == goal_index[0]) and (current.index[1] == goal_index[1]):
                if ignore_goal_orientation:
                    return self.reconstruct_path(current)
                elif (current.index[2] == goal_index[2]):
                    return self.reconstruct_path(current)

            # [new(xy, theta, cost, icp_xy), ...] [meter, rad]
            successors = self.get_successors(current.xy, current.rad)
            for stype, succ in successors:
                xy, rad, edge_weight, icp = succ
                xyt = (xy[0], xy[1], rad)

                # Collision check
                is_safe = not self.check_collision(xyt)
                if is_safe:
                    neighbor = self.grid.get_node(xyt)

                    steering_weight = steering_cost(current.steering_type, stype)
                    # print(">>{}".format(steering_weight))
                    tentative_g = current.g + edge_weight + steering_weight
                    # print("xyt:{:.3f}, {:.3f}, {:.3f}deg {}".format(xy[0], xy[1], np.degrees(rad), rad))
                    # print("ne-:{}, ten-g:{}".format(neighbor, tentative_g))

                    if tentative_g < neighbor.g:
                        neighbor.set_xyt(xyt)
                        neighbor.parent_index = current.index
                        neighbor.steering_type = stype
                        neighbor.set_score(tentative_g, heuristic(xyt))
                        if neighbor not in open_heap:
                            heapq.heappush(open_heap, neighbor)
                        else:
                            is_not_sorted = True
                        # print("ne-:{}, ten-g:{}".format(neighbor, tentative_g))
                        # print("op:{}".format(open_heap))
        # path_length, object[xyt,...], robot[xyt,...]
        return (0, [], [])

    def calc_robot_xyt(self, object_xyt):
        offset = (self.object_collision.width + self.robot_collision.width) / 2.0
        rt = object_xyt[2]
        rx = object_xyt[0] - offset * np.cos(rt)
        ry = object_xyt[1] - offset * np.sin(rt)
        return (rx, ry, rt)

    def reconstruct_path(self, last_node):
        reverse_path = [last_node]
        node = last_node
        while node.parent_index is not None:
            node = self.grid.get_node_from_index(node.parent_index)
            reverse_path.append(node)
        reverse_path.reverse()
        path = reverse_path

        path_length = len(path)
        object_path = [n.xyt for n in path]

        # robot path
        robot_path = [self.calc_robot_xyt(o_xyt) for o_xyt in object_path]
        return (path_length, object_path, robot_path)


if __name__ == "__main__":
    pass
