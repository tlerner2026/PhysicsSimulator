import math
from Variables import *

class StationKeepingController:
    """
    controller for station keeping behavior - manages keeping the boat within a defined area
    works in conjunction with the main Controller class but handles the specialized logic needed for drifting
    """

    def __init__(self, boat, waypoints, controller):
        self.boat = boat
        self.outer_waypoints = waypoints  # defines the outer boundary we must stay within
        self.state = "ENTERING"  # to be improved
        self.controller = controller
        
        # calculate center of boundary box for reference point
        self.center_x = sum(p[0] for p in waypoints) / len(waypoints)
        self.center_y = sum(p[1] for p in waypoints) / len(waypoints)
        
        # calculate inner box this is the target area we try to stay within to drift
        # inner box is 10m per side (20m total), scaled to degrees based on latitude
        box_half_size = 10  # meters
        dx = meter2degreeX(box_half_size, self.boat.refLat)
        dy = meter2degreeY(box_half_size)
        
        # define inner box corners relative to center
        self.inner_box = [
            [self.center_x+dx, self.center_y+dy],   # top right
            [self.center_x+dx, self.center_y-dy],   # bottom right
            [self.center_x-dx, self.center_y-dy],   # bottom left
            [self.center_x-dx, self.center_y+dy]    # top left
        ]

        # calculate upwind point
        self.upwind_target = self.calculate_upwind_point()
        
        self.return_path = []  # path for returning to upwind point after drifting
        self.target_path = []  # current target path being followed
        self.current_path_index = 0  # index in current path
        self.controller.course = []  # clear any existing course in main controller

    def update(self, dt):
        """
        the main control loop for station keeping behavior.
        implements a state machine for different phases of station keeping:
            note: for information on state machines see https://en.wikipedia.org/wiki/Finite-state_machine
        - ENTERING: moving from outside to inside the boundary box
        - REACHING_UPWIND: moving to the upwind target point
        - DRIFTING: drifting, with the goal of going along the wind angle
        - RETURNING: moving back to upwind point after drifting too far
        
        uses standard Controller path following for movement states (ENTERING, REACHING_UPWIND, RETURNING) but custom behavior for DRIFTING state

        Args:
            dt: time step in seconds
        """

        current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]
        
        # state machine logic
        if not self.is_in_box(self.outer_waypoints):
            # outside boundary box - need to enter
            self.state = "ENTERING"
            # use standard path planning to reach upwind target
            self.controller.course = [current_pos] + self.controller.leg(current_pos, self.upwind_target)
            self.target_path = self.controller.course[:]  # store for visualization; figure out how to erase paths after drawn
            self.current_path_index = 0

        elif self.state == "ENTERING":
            if self.is_in_box(self.outer_waypoints):
                # successfully entered box - now reach upwind point
                self.state = "REACHING_UPWIND"
                self.controller.course = [current_pos] + self.controller.leg(current_pos, self.upwind_target)
                self.target_path = self.controller.course[:]
                self.current_path_index = 0

        elif self.state == "REACHING_UPWIND":
            if self.at_point(self.upwind_target):
                # reached upwind point - start drifting
                self.state = "DRIFTING"
                # clear all path data as we stay in drifting state
                self.return_path = []
                self.target_path = []
                self.current_path_index = 0
                self.controller.course = []

                # this doesn't work, not sure why, will try to fix
                # Clear visualization of path 
                #for line in self.controller.boat.ax.get_lines():
                    #if line.get_color() == 'red':
                        #line.remove()

        elif self.state == "DRIFTING":
            if not self.is_near_upwind_point() and not self.is_in_box(self.inner_box):
                # drifted out of the interior bounding box, need to path back to upwind target
                self.state = "RETURNING"
                self.return_path = self.controller.leg(current_pos, self.upwind_target)
                self.controller.course = [current_pos] + self.return_path
                self.target_path = self.controller.course[:]
                self.current_path_index = 0
            else:
                # still drifting - point into wind to minimize speed
                wind_angle = self.boat.wind.angle.calc()
                # calculate a point 20m upwind to aim toward
                drift_dx = math.cos(math.radians(wind_angle)) * meter2degreeX(20, self.boat.refLat)
                drift_dy = math.sin(math.radians(wind_angle)) * meter2degreeY(20)
                drift_target = [current_pos[0] + drift_dx, current_pos[1] + drift_dy]
                self.controller.course = [current_pos, drift_target]

        elif self.state == "RETURNING":
            if self.at_point(self.upwind_target):
                # successfully returned to upwind point - resume drifting
                self.state = "DRIFTING"
                self.return_path = []
                self.target_path = []
                self.current_path_index = 0
                self.controller.course = []
                # clear visualization, also doesn't work, will try to fix
                for line in self.controller.boat.ax.get_lines():
                    if line.get_color() == 'red':
                        line.remove()
            elif len(self.return_path) > 0:
                # ppdate progress along return path
                if self.at_point(self.return_path[self.current_path_index]):
                    if self.current_path_index < len(self.return_path) - 1:
                        self.current_path_index += 1
                        self.controller.course = self.return_path[self.current_path_index:]

        # use main controller's methods for actual boat control
        self.controller.updateRudder(2, 1)
        self.controller.updateSails()

    def calculate_upwind_point(self): # this generally works i think, but it seems to break on corners for obvious reasons; will fix
        """
        calculates the optimal point in the inner box to aim for - the furthest point
        upwind; gives the most room to drift downwind while staying in the box
        
        the point is found by:
            1. explain this when it works properly and doesnt break on liek corners and whatever
        
        Returns:
            [x, y] coordinates of the target upwind point
        """
        # wind angle 270 means wind blowing south, add 180 to get upwind direction
        wind_angle = (self.boat.wind.angle.calc() + 180) % 360
        wind_rad = math.radians(wind_angle)
        
        # calculate unit vector in upwind direction
        wind_dx = math.cos(wind_rad)
        wind_dy = math.sin(wind_rad)

        # get inner box bounds for edge calculations
        min_x = min(p[0] for p in self.inner_box)
        max_x = max(p[0] for p in self.inner_box)
        min_y = min(p[1] for p in self.inner_box)
        max_y = max(p[1] for p in self.inner_box)

        # define box edges as line segments
        edges = [
            ((min_x, min_y), (max_x, min_y)),  # Bottom edge
            ((max_x, min_y), (max_x, max_y)),  # Right edge
            ((max_x, max_y), (min_x, max_y)),  # Top edge
            ((min_x, max_y), (min_x, min_y))   # Left edge
        ]

        # find all intersections of upwind direction with box edges
        intersections = []
        for edge in edges:
            intersection = self.line_intersection(
                (self.center_x, self.center_y),
                (self.center_x + wind_dx, self.center_y + wind_dy),
                edge[0],
                edge[1]
            )
            if intersection:
                intersections.append(intersection)

        # if no intersections found (shouldn't happen), use box center
        if not intersections:
            return [self.center_x, self.center_y]

        # select intersection point furthest upwind
        upwind_point = max(intersections, 
            key=lambda p: (p[0] - self.center_x) * wind_dx + (p[1] - self.center_y) * wind_dy)
        
        return list(upwind_point)

    def is_in_box(self, box_points):
        """
        check if boat is within a given box boundary
        the box must be defined by corner points and boat must be within x and y ranges
        
        Args:
            box_points: List of [x,y] coordinates defining box corners
        Returns:
            boolean indicating if boat is in box
        """
        x, y = self.boat.position.xcomp(), self.boat.position.ycomp()
        return (min(p[0] for p in box_points) <= x <= max(p[0] for p in box_points) and
                min(p[1] for p in box_points) <= y <= max(p[1] for p in box_points))

    def is_near_upwind_point(self, tolerance=3):
        """
        check if boat is within tolerance meters of upwind target point
        used to determine if we've drifted too far
        
        Args:
            tolerance: distance in meters
        Returns:
            boolean indicating if boat is near target
        """
        dx = self.upwind_target[0] - self.boat.position.xcomp()
        dy = self.upwind_target[1] - self.boat.position.ycomp()
        dist = math.sqrt(dx*dx + dy*dy)
        return degree2meter(dist) < tolerance

    def at_point(self, point, tolerance=2):
        """
        check if boat has reached a target point within tolerance
        used for waypoint arrival detection; BUT ONLY FOR "WAYPOINTS" THAT DONT ACTUALLY EXIST 
        
        Args:
            point: [x,y] coordinates of target
            tolerance: Distance in meters
        Returns:
            Boolean indicating if boat is at point
        """
        dx = point[0] - self.boat.position.xcomp()
        dy = point[1] - self.boat.position.ycomp()
        dist = math.sqrt(dx*dx + dy*dy)
        return degree2meter(dist) < tolerance

    def line_intersection(self, p1, p2, p3, p4):
        """
        calculate intersection point of two line segments.
        used for finding upwind target point.
        
        Args:
            p1, p2: Start and end points of first line
            p3, p4: Start and end points of second line
        Returns:
            (x,y) intersection point or None if no intersection
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denominator == 0:
            return None
            
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator
        
        if 0 <= t <= 1 and 0 <= u <= 1:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return (x, y)
        return None

    def printA(self, x):
        """
        normalize angle to [-180, 180] range
        display angles are compass angles afaik
        """
        x %= 360
        if x > 180:
            x = -180 + x-180
        return x

    def aoa(self, x):
        """
        calculate angle of attack
        """
        x = self.printA(x)
        return (44/90)*x