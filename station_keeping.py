import math
from Variables import *

class StationKeepingController:
    """
    Controller for station keeping behavior - manages keeping the boat within a defined area
    Works in conjunction with the main Controller class but handles the specialized logic needed for drifting
    """

    def __init__(self, boat, waypoints, controller, clear_paths_func):
        self.boat = boat
        self.outer_waypoints = waypoints
        self.state = "ENTERING"
        self.controller = controller
        self.clear_paths = clear_paths_func 
        
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
        
        uses standard Controller path following for movement states (ENTERING, REACHING_UPWIND, RETURNING) 
        but custom behavior for DRIFTING state
        """
        current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]
        
        current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]
        
        # State machine logic
        if not self.is_in_box(self.outer_waypoints):
            self.state = "ENTERING"
            self.target_path = self.controller.leg(current_pos, self.upwind_target)
            self.controller.course = [current_pos] + self.target_path
            print(f"ENTERING: {current_pos} -> {self.upwind_target}")

        elif self.state == "ENTERING":
            if self.is_in_box(self.outer_waypoints):
                self.state = "REACHING_UPWIND"
                self.target_path = self.controller.leg(current_pos, self.upwind_target)
                self.controller.course = [current_pos] + self.target_path
                print(f"REACHING_UPWIND: {current_pos} -> {self.upwind_target}")

        elif self.state == "REACHING_UPWIND":
            if not self.controller.course or len(self.controller.course) < 2:
                self.target_path = self.controller.leg(current_pos, self.upwind_target)
                self.controller.course = [current_pos] + self.target_path
                
            if self.at_point(self.upwind_target):
                self.state = "DRIFTING"
                self.target_path = []
                self.controller.course = []
                print("DRIFTING")

        elif self.state == "DRIFTING":
            if not self.is_near_upwind_point() and not self.is_in_box(self.inner_box):
                self.state = "RETURNING"
                self.target_path = self.controller.leg(current_pos, self.upwind_target)
                self.controller.course = [current_pos] + self.target_path
                print("RETURNING")

        elif self.state == "RETURNING":
            if not self.controller.course or len(self.controller.course) < 2:
                self.target_path = self.controller.leg(current_pos, self.upwind_target)
                self.controller.course = [current_pos] + self.target_path
                
            if self.at_point(self.upwind_target):
                self.state = "DRIFTING"
                self.target_path = []
                self.controller.course = []
                print("Back to DRIFTING")

        # Update visualization
        if hasattr(self.controller, 'display'):
            self.controller.display.clear_paths()
            if self.controller.course:  # Only plot if we have a course
                self.controller.display.boat.plotCourse(self.controller.course, 'red')

        # Update controls using controller's methods
        if self.state == "DRIFTING":
            # Get current wind angle and add 180° to point into wind
            target_angle = Angle(1, self.boat.wind.angle.calc() + 180)
            self.controller.updateRudderAngle(2, 1, target_angle)
        else:
            self.controller.updateRudder(2, 1)
        
        self.controller.updateSails()

        # Update visualization if needed
        # if self.clear_paths and self.controller.course:
            # self.clear_paths()
            # if hasattr(self.controller, 'display'):
                # self.controller.display.boat.plotCourse(self.controller.course, 'red')

    def calculate_upwind_point(self):
        """
        Calculate upwind point opposite to wind direction.
        For 90° wind (from north), this will be the top point of circle (270°).
        """
        #Add 180° to wind angle to get upwind direction
        #wind_angle = (self.boat.wind.angle.calc() + 180) % 360

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