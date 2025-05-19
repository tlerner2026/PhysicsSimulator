from station_keeping import StationKeepingController
import math
from Variables import *
import numpy as np

def printA(x):
    """
    This takes an angle and turns it into a + or - angle, so it is between -180 and +180
    """
    x %= 360
    if x > 180:
        x = -180 + x-180
    return x

def aoa(x):
    """
    Angle of Attack, takes the angle of the boat and turns it into the angle of the foil???
    """
    x = printA(x)
    # if x < 0:
    #     return -44/90*x
    return (44/90)*x
    # return -0.5*x+44#4/9

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.last_error = None

    def update(self, error, dt):
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        D = 0
        if self.last_error is not None:
            D = self.Kd * (error - self.last_error) / dt
        self.last_error = error
        return P + I + D

class Controler():

    def __init__(self, Boat, polars="test.pol"):
        self.boat = Boat
        self.polars = self.readPolar(polars)
        self.waypoints = []  # original waypoints from course
        self.active_course = []  # current active leg pairs being followed
        self.wind_change_points = []  # track points added due to wind changes
        self.current_target_idx = 0  # index of next target in original waypoints
        self.initial_wind_angle = (Angle(1,180)-((self.boat.wind.angle-self.boat.linearVelocity.angle)*-1)).calc()
        self.control_mode = "normal"
        self.courseType = None  # store course type
        self.station_keeper = None  # initialize station keeper as None
        self.rudder_pid = PIDController(Kp=70.0, Ki= 0.5, Kd=35.0) #KP max = 62, 
        self.heading_error_scaled = 0
        self.heading_error = 0
        self.count = 0

    def recalculate_path(self):
        
        """
        Recalculate path including the next waypoint we were heading towards
        """
        # current_wind = self.boat.wind.angle.calc()
        # wind_diff = abs(printA(current_wind - self.initial_wind_angle))


        if self.control_mode != "station_keeping":
            # get current position
            current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]
            self.active_course = self.wind_change_points.copy()
        
            # add current position if we don't have any points yet
            if not self.active_course:
                self.active_course.append(current_pos)
            
            # find which waypoint we were heading to
            target_idx = 0
            min_dist = float('inf')
            current_course_idx = 0
            
            # first find where we are in the current course
            for i, point in enumerate(self.active_course):
                dist = math.sqrt((point[0] - current_pos[0])**2 + (point[1] - current_pos[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    current_course_idx = i
            
            # then find the next waypoint we were heading to
            for i, waypoint in enumerate(self.waypoints):
                # check if this waypoint appears in the course after our current position
                for point in self.active_course[current_course_idx:]:
                    if abs(point[0] - waypoint[0]) < 1e-6 and abs(point[1] - waypoint[1]) < 1e-6:
                        target_idx = i
                        break
                if target_idx == i:  # if we found our target, stop searching
                    break
            
            # create new course starting from current position
            new_course = [current_pos]
            
            # calculate path to the next waypoint
            next_leg = self.leg(current_pos, self.waypoints[target_idx])
            new_course.extend(next_leg)
            
            # then continue with remaining waypoints
            for i in range(target_idx + 1, len(self.waypoints)):
                next_leg = self.leg(new_course[-1], self.waypoints[i])
                new_course.extend(next_leg)
            
            # replace entire course with new calculation
            self.active_course = new_course
            
            # self.initial_wind_angle = current_wind
            return True
                
        return False

    def plan(self, plantype, waypoints):
        """Initial course planning"""
        self.waypoints = waypoints  # store original waypoints
        self.current_target_idx = 0
        self.wind_change_points = []
        self.courseType = plantype  # store course type for reference
        
        if plantype == "e":  # endurance
            self.control_mode = "normal"
            self.station_keeper = None
            # calculate first two legs only
            self.calculate_next_legs()
            return self.active_course
            
        elif plantype == "s":  # station keeping
            self.control_mode = "station_keeping"
            # initialize station keeper
            self.station_keeper = StationKeepingController(
                self.boat, 
                waypoints, 
                self,
                self.display.clear_paths if hasattr(self, 'display') else None
            )
            # get current position and calculate path to upwind point
            current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]
            self.active_course = [current_pos] + self.leg(current_pos, self.station_keeper.upwind_target)
            return self.active_course
            
        elif plantype == "p":  # precision
            self.control_mode = "normal"
            self.station_keeper = None
            # calculate first two legs only
            self.calculate_next_legs()
            return self.active_course
        
    def calculate_next_legs(self):
        """Calculate next two legs of the course"""
        current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]

       
        # clear active course but keep wind change points
        self.active_course = self.wind_change_points.copy()
        
        # add current position if we don't have any points yet
        if not self.active_course:
            self.active_course.append(current_pos)
        
        # calculate path to next target
        if self.current_target_idx < len(self.waypoints):
            next_target = self.waypoints[self.current_target_idx % len(self.waypoints)]
            next_leg = self.leg(self.active_course[-1], next_target)
            self.active_course.extend(next_leg)
            
            # calculate one more leg if we need to loop back to start for endurance
            if self.courseType == "e" and self.current_target_idx == len(self.waypoints) - 1:
                next_leg = self.leg(self.active_course[-1], self.waypoints[0])
            elif self.current_target_idx + 1 < len(self.waypoints):
                next_leg = self.leg(self.active_course[-1], self.waypoints[(self.current_target_idx + 1) % len(self.waypoints)])
            else:
                next_leg = []
            
            self.active_course.extend(next_leg)
        

        # update visualization if display exists
        if hasattr(self, 'display'):
            self.display.clear_paths()
            self.display.boat.plotCourse(self.active_course, 'green')

    def leg(self, start, stop):
        """ 
        The purpose of leg is when given a start point and stop point it checks if the straight line
        involves going upwind or downwind. If it is upwind or downwind it creates an intermediate point
        based on the boats polars to allow for tacking or jibbing. 
        Between two buoys it only does a single tack.
        Waypoints are buoys.
        """
        angle = Angle(1,math.atan2(stop[1]-start[1],stop[0]-start[0])*180/math.pi)
        apparentAngle = abs(printA(Angle.norm(self.boat.wind.angle+Angle(1,180)-angle).calc()))
        
        relative_wind = abs(printA(Angle.norm(Angle(1,180)-((self.boat.wind.angle-self.boat.linearVelocity.angle)*-1)).calc()))

        global_wind = (self.boat.linearVelocity.angle - Angle(1,180)-((self.boat.wind.angle-self.boat.linearVelocity.angle)*-1))

        #######    print(Angle.norm(self.boat.wind.angle-self.boat.linearVelocity.angle - angle).calc())

        # apparent angle is the wind angle relative to the boat between -180 and +180 
        # get the last element of self.polars
        if (relative_wind < 53):
        ##### if apparentAngle < self.polars[-1][0]: # upwind
            # We want to get to stop only using the upwind BVMG
            """
            we want to get to stop only using the upwind BVMG
            variables:
            - v: vector representing the direction and distance from start to stop
                - Angle(...) calculates the angle in degrees
                - math.sqrt(...) calculates the Euclidean distance
            - k and j: vectors representing optimal tacking directions relative to the wind
            - D, Dk, Dj: determinants that solve the linear system, finding how much of k and j vectors are needed to account for the winds effect while reaching the destination
            - a, b: scaling factors for vectors k and j respectively, determining how much the boat should follow each vector (tacking directions)

            returns the number of steps along the k vector to reach the intermediate point
            """
            v = Vector(Angle(1,round(math.atan2(stop[1]- start[1],stop[0]- start[0])*180/math.pi*10000)/10000),math.sqrt((stop[0]- start[0])**2+(stop[1]- start[1])**2))
            k = Vector(global_wind+Angle(1,180+self.polars[-1][0]),1) # to the right of the no sail zone
            j = Vector(global_wind+Angle(1,180-self.polars[-1][0]),1) # to the left of the no sail zone
            D = np.linalg.det(np.array([[k.xcomp(),j.xcomp()],[k.ycomp(),j.ycomp()]]))
            Dk = np.linalg.det(np.array([[v.xcomp(),j.xcomp()],[v.ycomp(),j.ycomp()]]))
            Dj = np.linalg.det(np.array([[k.xcomp(),v.xcomp()],[k.ycomp(),v.ycomp()]]))
            a = Dk/D # number of k vectors; aka how far to sail along k
            b = Dj/D # number of j vectors; aka how far to sail along j
            k.norm *= a
            j.norm *= b
            # calculates the ideal tacking point
            # calculates the ideal intermediate point between start and end if you are going upwind
            ans = [[start[0]+k.xcomp(),start[1]+k.ycomp()],stop] # returns steps on k vector
            return  ans
        #elif apparentAngle > self.polars[-1][1]: #downwind
        elif (relative_wind > 127):
            """
            variables:
            - v: vector representing the direction and distance from start to stop
                - Angle(...) calculates the angle in degrees
                - math.sqrt(...) calculates the Euclidean distance
            - k and j: vectors representing two optimal gybing directions based on the wind
            - D, Dk, Dj: determinants that solve the linear system, finding how much of k and j vectors are needed to account for the winds effect while reaching the destination
            - a, b: scaling factors for vectors k and j respectively, determining how much the boat should follow each vector (gybing directions)

            returns the number of steps along the k vector to reach the intermediate point
            """
            v = Vector(Angle(1,round(math.atan2(stop[1]- start[1],stop[0]- start[0])*180/math.pi*10000)/10000),math.sqrt((stop[0]- start[0])**2+(stop[1]- start[1])**2))
            k = Vector(global_wind+Angle(1,180+self.polars[-1][1]),1)
            j = Vector(global_wind+Angle(1,180-self.polars[-1][1]),1)
            D = np.linalg.det(np.array([[k.xcomp(),j.xcomp()],[k.ycomp(),j.ycomp()]]))
            Dk = np.linalg.det(np.array([[v.xcomp(),j.xcomp()],[v.ycomp(),j.ycomp()]]))
            Dj = np.linalg.det(np.array([[k.xcomp(),v.xcomp()],[k.ycomp(),v.ycomp()]]))
            a = Dk/D # number of k vectors
            b = Dj/D # number of j vectors
            k.norm *= a
            j.norm *= b
            # calculates the ideal jibbing point
            # calculates the ideal intermediate point between start and end if you are going downwind
            ans = [[start[0]+k.xcomp(),start[1]+k.ycomp()],stop]
            return  ans

        # if the straightline doesnt go upwind or downwind it returns the end buoy
        return [stop]
    
    # NOTE: I've desided using best course to next mark while probably the optimal solution brings in a level of complexity that we do not
    # have the time to handle, thus we'll be simplifying.
    # def BestCNM(self, angle, wind): # best course to next mark
    #     # angle is relative to wind
    #     ma = 0
    #     mcnm = 0
    #     for a in range(-180,180):
    #         l = self.VB(Angle(1,a), wind)
    #         CNM = Vector(Angle(1,a),l) * Vector(angle,l)
    #         if mcnm < CNM:
    #             ma = a
    #             mcnm = CNM
    #     axis  = printA(angle.calc())
    #     return [ma,ma-(ma - axis)*2]

    def handle_wind_change(self):
        """Handle significant wind changes by recalculating path from current position"""

        if self.control_mode != "station_keeping":
            current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]
            
            # add current position to wind change points
            self.wind_change_points.append(current_pos)
            
            # recalculate next legs from current position
            self.calculate_next_legs()
            
            return True
        return False

    def VB(self,angle, wind): # reading boat polars
        angle =abs(angle.calc())
        angle %= 180
        for i, a in enumerate(self.polars[1:-1]):
            if a[0] > angle:
                for j, s in enumerate(self.polars[0][1:]):
                    if s > wind:
                        return self.polars[i+1][j+1] #TODO add interpolation
        return -1
            

    def readPolar(self,polar):
        rtn =[]
        text = open(polar).read().split('\n')
        c = "\t"
        if text[0].find(";") != -1:
            c = ";"
        rtn.append([0]+[float(x) for x in text[0].split(c)[1:]])
        for i in text[1:-1]:
            if i.split(c)[0] != '':
                rtn.append([float(x) for x in i.split(c)])
        rtn.append([float(x) for x in text[-1].split(";")[1:]])
        # print(rtn) # prints a list corresponding to a boat angle relative to wind of lists of speeds corresponding to wind speeds
        return rtn

    def update(self, dt, rNoise=2, stability=1):
        """Main update loop"""
        if self.control_mode == "station_keeping" and self.station_keeper is not None:
            # give full control to station keeper
            self.station_keeper.update(dt)
        else:
            # normal course following logic
            self.count += 1
            if self.target_reached():
                self.wind_change_points = []
                
                if self.courseType == "e" and self.current_target_idx >= len(self.waypoints) - 1:
                    self.current_target_idx = 0
                    self.count = 0
                    print("Completed lap, continuing endurance course")
                else:
                    self.current_target_idx += 1
                
                self.calculate_next_legs()
                
            elif (self.count == 30):
                self.count = 0
                self.calculate_next_legs()
                pass
            
            self.updateRudder(dt, rNoise, stability)
            self.updateSails()

    def isEnp(self,a1,a2): # Enp means something like "Enroulement de Point" (French for "gybe point")
        """
        Checks if the wind direction lies between two angles to check if gybing is necessary
        Returns True if gybing is necessary, and False if it is not
        """
        a1 = Angle.norm(a1).calc()
        a2 = Angle.norm(a2).calc()
        wind = Angle.norm(self.boat.wind.angle).calc()
        if (a1 < wind and wind < a2) or (a2 < wind and wind < a1):
            return True
        return False

    def isVir(self,a1,a2): # Vir means something like "Virer" (French for "to tack")
        """
        Checks if the wind direction (reversed by 180 deg) lies between two angles to check if tacking is necessary
        Returns True if tacking is necessary, and False if it is not
        """
        a1 = Angle.norm(a1).calc()
        a2 = Angle.norm(a2).calc()
        wind = Angle.norm(self.boat.wind.angle+Angle(1,180)).calc()
        if (a1 < wind and wind < a2) or (a2 < wind and wind < a1):
            return True
        return False

    def nextP(self):
        """Determines when the boat has reached the next point on its course"""
        # Ensure active_course has at least two points
        if not self.active_course or len(self.active_course) < 2:
            current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]
            self.active_course = [current_pos, current_pos]
            return 0

        r = 5  # meters radius
        
        dy = (self.boat.position.ycomp() - self.active_course[0][1])
        dx = (self.boat.position.xcomp() - self.active_course[0][0])
        dist = degree2meter(math.sqrt(dx**2 + dy**2))
        
        if dist < r:
            a1 = Angle(1, round(math.atan2(dy, dx)*180/math.pi*10000)/10000)
            a2 = Angle(1, round(math.atan2(self.active_course[1][1] - self.active_course[0][1],
                                        self.active_course[1][0] - self.active_course[0][0])*180/math.pi*10000)/10000)
            self.active_course.pop(0)
            
            if len(self.active_course) == 1:  # Ensure we always have 2 points
                self.active_course.append(self.active_course[0])
                
            if self.isEnp(a1, a2):  # gybe necessary
                return 1
            if self.isVir(a1, a2):  # tacking necessary
                return 2
        return 0

    def target_reached(self):
        """Check if current target waypoint has been reached"""
        if self.current_target_idx >= len(self.waypoints) and self.courseType != "e":
            return False
            
        current_target = self.waypoints[self.current_target_idx % len(self.waypoints)]
        current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]
        
        # Check if we're within 5 meters of target
        dx = current_target[0] - current_pos[0]
        dy = current_target[1] - current_pos[1]
        dist = degree2meter(math.sqrt(dx**2 + dy**2))
        
        if dist < 5:  # 5 meter radius for reaching target
            print(f"Reached target {self.current_target_idx % len(self.waypoints)}")
            return True
        return False


    def updateRudder(self, dt, rNoise, stability):
        if not self.active_course or len(self.active_course) < 2:
            current_pos = [self.boat.position.xcomp(), self.boat.position.ycomp()]
            self.active_course = [current_pos, current_pos]

        self.nextP()

        dx = self.active_course[0][0] - self.boat.position.xcomp()
        dy = self.active_course[0][1] - self.boat.position.ycomp()
        target_angle = Angle(1, math.atan2(dy, dx) * 180 / math.pi)
        current_angle = self.boat.linearVelocity.angle
        self.heading_error = printA((target_angle - current_angle).calc())

        self.heading_error_scaled = self.heading_error / 180.0

        rudder_correction = self.rudder_pid.update(self.heading_error_scaled, dt)
        rudder_angle = max(min(rudder_correction, 20), -20) * -1


        dtheta = (target_angle - current_angle).calc()
        rotV = self.boat.rotationalVelocity*180/math.pi *0.03
        dtheta = printA(dtheta)
        coeff = 2/math.pi * math.atan((dtheta)/40 - rotV/stability)
        self.boat.hulls[-1].angle = Angle(1,-10*coeff)*rNoise
        

        # No rNoise scaling here
       #self.boat.hulls[-1].angle = Angle(1, rudder_angle)


    
    def updateRudderAngle(self,rNoise,stability,angle):
        """
        Updates the rudder's angle based on a given target angle rather then calculating it from the boat's position and course
        """
        target_angle = angle
        current_angle = self.boat.linearVelocity.angle
        dtheta = (target_angle - current_angle).calc() # difference in current angle and target angle

        rotV = self.boat.rotationalVelocity*180/math.pi *0.03 # rotational velocity * (180/pi) converts from radians to degrees. *0.03 is for time step conversion.

        # coeff = 1-(1/(dtheta.calc()*(1/rotV)+1))
        dtheta = printA(dtheta)

        """
        step by step logic for coeff
        1. (dtheta)/40 -> Normalized heading error; prevents large heading errors from causing excessively large control inputs
        2. rotV/stability -> Normalized rotational velocity
        3. (Normalized heading error - Normalized rotation velocity) -> combines desired change and current rotational velocity to create a single value for the control law
        4. arctan(control law value) -> smoothly bounded output val
        5. (output val) * (2/pi) -> normalizes range from +/-(pi/2) to a more convient control range +/-1
        """
        coeff = 2/math.pi * math.atan((dtheta)/40 - rotV/stability) # determines how aggressively the rudder should be adjusted to steer the boat towards the target angle

        self.boat.hulls[-1].angle = Angle(1,-10*coeff)*rNoise  # * -10 converts the coefficent into a rudder angle adjustment within the effective range for the rudder's physical constraints
    
    def updateSails(self):
        """
        Updates the angle of the boat's sails based on the apparent wind direction
            * The active code simplifies the calculation by directly adjusting wind angle
            * The commented out code tries to perform a more percise calculation:
                * Should be theoretically more optimal but not necessary
        """
        #angle = Angle.norm(self.boat.angle + Angle(1,90)-self.boat.globalAparentWind().angle+Angle(1,180)).calc()
        wind = self.boat.globalAparentWind().angle
        wind += Angle(1,180)
        wind = wind - self.boat.angle


        # # angle = Angle(1,math.acos((wind * Vector(self.boat.angle,1))/wind.norm)*180/math.pi)
        # # if Angle.norm(wind.angle+Angle(1,180)).calc() > angle.calc():
        # #     angle = Angle(1,180) -angle
        # # angle = angle.calc()
        # self.boat.sails[0].setSailRotation(Angle(1,aoa(angle)))

        self.boat.sails[0].setSailRotation(Angle(1,aoa(wind.calc()))) # adjusts the sails to point at the near optimal angle