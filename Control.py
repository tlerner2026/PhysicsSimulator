# I'll start out with a simple test that aims to maintain a certain direction
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

class Controler():
    # def __init__(self,Boat, polars = "MarPol.pol"):   # Original Constructor
        # self.boat = Boat
        # self.polars = self.readPolar(polars)
        # self.course = []

    def __init__(self,Boat, polars = "test.pol"): # test.pol or MarPol.pol?
        """
        Intialize the boat, polars, and course of the boat.
        """
        self.boat = Boat # sets boat to object of boat class
        self.polars = self.readPolar(polars) 
        self.course = []

    def plan(self,plantype,waypoints):
        """
        Plan: plot an ideal course for the boat to take given the event type and buoy waypoints
        """
        # Type of self.boat.position is Vector.
        course = [[self.boat.position.xcomp(),self.boat.position.ycomp()]]# Course will comprise of a sequence of checkpoints creating a good path
        #type can either E(ndurance), S(tation Keeping), p(recision Navigation), w(eight/payload),
        if plantype == "e":#endurance
            # Format of waypoints is as such
            # 4 Buoy in order of navigation
            n = 4
            course.extend(self.leg([self.boat.position.xcomp(), self.boat.position.ycomp()], waypoints[0]))
            # course is a list of xcomps and ycomps. This is computed using leg function
            
            #code to do one repetition of the course:
            c2 = self.leg(waypoints[0],waypoints[1])
            c2.extend(self.leg(waypoints[1],waypoints[2]))
            c2.extend(self.leg(waypoints[2],waypoints[3]))
            c2.extend(self.leg(waypoints[3],waypoints[0]))
            #append for laps of the course to the main course list 
            course.extend(c2*n)
            #remove the last step since you don't need to return to the start buoy, just the dock at the end
            course.pop()
            #return to the dock: 
            course.extend(self.leg(waypoints[3],[self.boat.position.xcomp(), self.boat.position.ycomp()]))
        elif plantype == "s":#station keeping
            """
            deprecated
            """
            #4 Buoy in any order
            # center
            # course.extend(self.leg([self.boat.position.xcomp(), self.boat.position.ycomp()], [sum(p[0] for p in waypoints)/len(waypoints),sum(p[1] for p in waypoints)/len(waypoints)]))
            last = [self.boat.position.xcomp(), self.boat.position.ycomp()]
            for i in range(-1,4):
                avg = [(waypoints[i%4][0]+waypoints[(i+1)%4][0])/2,(waypoints[i%4][1]+waypoints[(i+1)%4][1])/2]
                angle = math.atan2((waypoints[i%4][1]-waypoints[(i+1)%4][1]),(waypoints[i%4][0]-waypoints[(i+1)%4][0]))+math.pi/2
                l = math.sqrt((waypoints[(i+1)%4][1]-waypoints[(i+2)%4][1])**2+(waypoints[(i+1)%4][0]-waypoints[(i+2)%4][0])**2)/8
                avg[0] -= math.cos(angle)*l
                avg[1]-= math.sin(angle)*l
                course.extend(self.leg(last, avg))
                last = avg
            # course.extend(self.leg(waypoints[1],waypoints[2]))
            # course.extend(self.leg(waypoints[2],waypoints[3]))
            # course.extend(self.leg(waypoints[3],waypoints[0]))
        elif plantype == "p":#precision
            # 4 Buoy in order of navigation'
            course.extend(self.leg([self.boat.position.xcomp(), self.boat.position.ycomp()], waypoints[1]))
            course.extend(self.leg(waypoints[1],waypoints[2]))
            course.extend(self.leg(waypoints[2],[(waypoints[0][0]+waypoints[3][0])/2,(waypoints[0][1]+waypoints[3][1])/2]))
        return course

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
        # apparent angle is the wind angle relative to the boat between -180 and +180 
        # get the last element of self.polars
        if apparentAngle < self.polars[-1][0]: # upwind
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
            """
            v = Vector(Angle(1,round(math.atan2(stop[1]- start[1],stop[0]- start[0])*180/math.pi*10000)/10000),math.sqrt((stop[0]- start[0])**2+(stop[1]- start[1])**2))
            k = Vector(self.boat.wind.angle+Angle(1,180+self.polars[-1][0]),1)
            j = Vector(self.boat.wind.angle+Angle(1,180-self.polars[-1][0]),1)
            D = np.linalg.det(np.array([[k.xcomp(),j.xcomp()],[k.ycomp(),j.ycomp()]]))
            Dk = np.linalg.det(np.array([[v.xcomp(),j.xcomp()],[v.ycomp(),j.ycomp()]]))
            Dj = np.linalg.det(np.array([[k.xcomp(),v.xcomp()],[k.ycomp(),v.ycomp()]]))
            a = Dk/D # number of k vectors
            b = Dj/D # number of j vectors
            k.norm *= a
            j.norm *= b
            # calculates the ideal tacking point
            # calculates the ideal intermediate point between start and end if you are going upwind
            ans = [[start[0]+k.xcomp(),start[1]+k.ycomp()],stop]
            return  ans
        elif apparentAngle > self.polars[-1][1]: #downwind
            """
            variables:
            - v: vector representing the direction and distance from start to stop
                - Angle(...) calculates the angle in degrees
                - math.sqrt(...) calculates the Euclidean distance
            - k and j: vectors representing two optimal gybing directions based on the wind
            - D, Dk, Dj: determinants that solve the linear system, finding how much of k and j vectors are needed to account for the winds effect while reaching the destination
            - a, b: scaling factors for vectors k and j respectively, determining how much the boat should follow each vector (gybing directions)
            """
            v = Vector(Angle(1,round(math.atan2(stop[1]- start[1],stop[0]- start[0])*180/math.pi*10000)/10000),math.sqrt((stop[0]- start[0])**2+(stop[1]- start[1])**2))
            k = Vector(self.boat.wind.angle+Angle(1,180+self.polars[-1][1]),1)
            j = Vector(self.boat.wind.angle+Angle(1,180-self.polars[-1][1]),1)
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


    def update(self,dt,rNoise= 2,stability=1): # less noise = faster rotation, stability tries to limit angular momentum
        """
        Updates the rudder and sail to maintain course stability, adjusted on noise and stability parameters
        """
        self.updateRudder(rNoise,stability)
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
        """
        Determines when the boat has reached the next point on its course and checks if a tack or gybe is necessary based on wind direction
        """
        r = 5# the distance from boat to waypoint to qualify as having reached it; 7 meter radius to play it safe

        dy = (self.boat.position.ycomp() - self.course[0][1])
        dx = (self.boat.position.xcomp() - self.course[0][0])
        dist = degree2meter(math.sqrt(dx**2 + dy**2)) # Euclidean distance from boat to buoy
        if dist < r: # if the actual distance between boat and buoy is less then r, the waypoint has been reached
            a1 =Angle(1,round(math.atan2(dy,dx)*180/math.pi*10000)/10000) # angle between boat and current waypoint
            a2 =Angle(1,round(math.atan2(self.course[1][1]- self.course[0][1],self.course[1][0]-self.course[0][0])*180/math.pi*10000)/10000) # angle between the current waypoint and next waypoint
            self.course.pop(0) # removes current waypoint from course
            if self.isEnp(a1,a2): # gybe necessary
                return 1
            if self.isVir(a1,a2): # tacking necessary
                return 2
        return 0 # no gybe or tack necessary

    def updateRudder(self,rNoise,stability):
        """
        Updates the rudder's angle in order to adjust the boat's heading
        """
        self.nextP() # gybe/tack check

        if self.course[0][0] == -1: 
            #mise à la cape sous GV

            """
            Mise à la cape - heaving to
            sous - under
            GV - Grand Voile - mainsail

            mise à la cape sous GV - heaving to under mainsail

            Seems like this is where Henri intended to have certain conditions result in the boat stopping
            movement, like what we planned to do in station keeping
            """
            pass

        dx = self.course[0][0]-self.boat.position.xcomp()
        dy = self.course[0][1]-self.boat.position.ycomp()
        target_angle = Angle(1,math.atan2(dy,dx)*180/math.pi) # angle between boat and waypoint; aka the target angle
        #target_angle = angle
        current_angle = self.boat.linearVelocity.angle # current angle of boats velocity
        dtheta = (target_angle - current_angle).calc() # difference in current angle and target angle
        rotV = self.boat.rotationalVelocity*180/math.pi *0.03 
        # coeff = 1-(1/(dtheta.calc()*(1/rotV)+1))
        dtheta = printA(dtheta)
        coeff = 2/math.pi * math.atan((dtheta)/40 - rotV/stability) # adjust rudder angle based on dtheta, rotational velocity, and stability (stability is a smoothing factor)
        self.boat.hulls[-1].angle = Angle(1,-10*coeff)*rNoise # set new rudder angle
    
    def updateRudderAngle(self,rNoise,stability,angle):
        """
        Updates the rudder's angle based on a given target angle rather then calculating it from the boat's position and course
        """
        target_angle = angle
        current_angle = self.boat.linearVelocity.angle
        dtheta = (target_angle - current_angle).calc()
        rotV = self.boat.rotationalVelocity*180/math.pi *0.03
        # coeff = 1-(1/(dtheta.calc()*(1/rotV)+1))
        dtheta = printA(dtheta)
        coeff = 2/math.pi * math.atan((dtheta)/40 - rotV/stability)
        self.boat.hulls[-1].angle = Angle(1,-10*coeff)*rNoise
    
    def updateSails(self):
        """
        Updates the angle of the boat's sails based on the apparent wind direction
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
        self.boat.sails[0].setSailRotation(Angle(1,aoa(wind.calc())))