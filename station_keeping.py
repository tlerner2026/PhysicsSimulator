import math
from Variables import *

class StationKeepingController:
    """
    Controller for station keeping behavior - manages keeping the boat within a defined area
    Works in conjunction with the main Controller class but handles the specialized logic needed for drifting
    """

    def __init__(self, boat, waypoints, controller):
        self.boat = boat
        self.outer_waypoints = waypoints  # defines the outer boundary we must stay within
        self.state = "ENTERING"  # to be improved
        self.controller = controller
        
        # Calculate center of boundary box for reference point
        self.center_x = sum(p[0] for p in waypoints) / len(waypoints)
        self.center_y = sum(p[1] for p in waypoints) / len(waypoints)
        
        # Calculate inner box dimensions - this defines our drift radius
        # Inner box is 10m per side (20m total), scaled to degrees based on latitude
        box_half_size = 10  # meters
        self.drift_radius_deg_x = meter2degreeX(box_half_size, self.boat.refLat)
        self.drift_radius_deg_y = meter2degreeY(box_half_size)
        
        # For visualization purposes, still maintain inner box corners
        self.inner_box = [
            [self.center_x + self.drift_radius_deg_x, self.center_y + self.drift_radius_deg_y],
            [self.center_x + self.drift_radius_deg_x, self.center_y - self.drift_radius_deg_y],
            [self.center_x - self.drift_radius_deg_x, self.center_y - self.drift_radius_deg_y],
            [self.center_x - self.drift_radius_deg_x, self.center_y + self.drift_radius_deg_y]
        ]

        # Calculate upwind point
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
                # Drifted too far, return to upwind point
                self.state = "RETURNING"
                self.return_path = self.controller.leg(current_pos, self.upwind_target)
                self.controller.course = [current_pos] + self.return_path
                self.target_path = self.controller.course[:]
                self.current_path_index = 0
            else:
                # Get wind angle and flip it 180° to point into wind
                wind_angle = (self.boat.wind.angle.calc() + 180) % 360
                
                # Set drift target in direction facing into wind
                drift_dx = math.cos(math.radians(wind_angle)) * meter2degreeX(20, self.boat.refLat)
                drift_dy = math.sin(math.radians(wind_angle)) * meter2degreeY(20)
                drift_target = [current_pos[0] + drift_dx, current_pos[1] + drift_dy]
                self.controller.course = [current_pos, drift_target]

                # tentative rudder updates:
                self.controller.updateRudder(4, 0.5)  # More aggressive rudder control
                self.controller.updateSails()
                return 

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

    def calculate_upwind_point(self):
        """
        Calculate upwind point opposite to wind direction.
        For 90° wind (from north), this will be the top point of circle (270°).
        """
        # Add 180° to wind angle to get upwind direction
        # wind_angle = (self.boat.wind.angle.calc() + 180) % 360

        # don't add 180° to start at the bottom of the circle facing toward the wind
        wind_angle = (self.boat.wind.angle.calc()) % 360


        wind_rad = math.radians(wind_angle)
        
        # Calculate point on circle in upwind direction
        point_x = self.center_x + (self.drift_radius_deg_x * math.cos(wind_rad))
        point_y = self.center_y + (self.drift_radius_deg_y * math.sin(wind_rad))
        
        return [point_x, point_y]

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

    def is_near_upwind_point(self, tolerance=3): # 3 potentially?
        """
        Check if boat is within tolerance meters of upwind target point.
        Used to determine if we've drifted too far.
        
        Args:
            tolerance: distance in meters
        Returns:
            boolean indicating if boat is near target
        """
        # Calculate distance from center
        dx = (self.boat.position.xcomp() - self.center_x) / self.drift_radius_deg_x
        dy = (self.boat.position.ycomp() - self.center_y) / self.drift_radius_deg_y
        
        # Normalize to account for lat/lon scaling
        dist = math.sqrt(dx*dx + dy*dy)
        
        # If distance from center is greater than 1 radius plus tolerance,
        # we've drifted too far
        return dist <= 1.0 + (tolerance / 10.0)  # Divide by 10 since radius is 10m

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