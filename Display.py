#Display and Map
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.transforms as transforms
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import InsetPosition
from matplotlib.widgets import Slider, Button
from Map import regionPolygon, loadGrib
#Boat and variables
from Foil import foil, Winch
from Variables import *
from Boat import Boat
from Control import Controler, printA
from Compressor import *
from station_keeping import StationKeepingController
#Other
import os
import math
import copy
import re

fps = 70
numCycle = 1
data_dir = os.path.dirname(__file__) #abs dir

def rm_ansi(line):
    ansi_escape = re.compile(r'(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]')
    return ansi_escape.sub('', line)

class boatDisplayShell():
    def __init__(self,Boat,ax,refLat):
        self.boat = Boat
        self.ax = ax
        self.refLat = refLat
        self.target_point = None
        self.inner_waypoints = None
        self.courseType = "e"  # Set course type
        self.autopilot = None  # Initialize autopilot as None
        self.wind_arrow = None

    def initAuto(self):
        # self.courseType = "s" # for testing, forces mode

        if self.courseType == "p":  # Precision navigation
            self.waypoints = [
                [self.boat.position.xcomp()-meter2degreeX(1.5,self.refLat),self.boat.position.ycomp()],
                [self.boat.position.xcomp()-meter2degreeX(1.5,self.refLat)-meter2degreeX(25,self.refLat),self.boat.position.ycomp()-meter2degreeY(25*math.sqrt(3))],
                [self.boat.position.xcomp()+meter2degreeX(1.5,self.refLat)+meter2degreeX(25,self.refLat),self.boat.position.ycomp()-meter2degreeY(25*math.sqrt(3))],
                # [self.boat.position.xcomp()+meter2degreeX(3,self.refLat),self.boat.position.ycomp()+meter2degreeY(3*math.sqrt(3))],
                # [self.boat.position.xcomp(),self.boat.position.ycomp()+meter2degreeY(5*math.sqrt(3))],
                [self.boat.position.xcomp()+meter2degreeX(1.5,self.refLat),self.boat.position.ycomp()],
            ]
        elif self.courseType == "e":  # Endurance
            self.waypoints = [

                # [self.boat.position.xcomp()-meter2degreeX(25,self.refLat),self.boat.position.ycomp()],
                # [self.boat.position.xcomp()-meter2degreeX(25,self.refLat),self.boat.position.ycomp()+meter2degreeY(15)],
                # [self.boat.position.xcomp()+meter2degreeX(25,self.refLat),self.boat.position.ycomp()+meter2degreeY(15)],
                # [self.boat.position.xcomp()+meter2degreeX(25,self.refLat),self.boat.position.ycomp()],
                
                [self.boat.position.xcomp()+meter2degreeX(25,self.refLat),self.boat.position.ycomp()],
                [self.boat.position.xcomp()+meter2degreeX(25,self.refLat),self.boat.position.ycomp()+meter2degreeY(15)],
                [self.boat.position.xcomp()-meter2degreeX(25,self.refLat),self.boat.position.ycomp()+meter2degreeY(15)],
                [self.boat.position.xcomp()-meter2degreeX(25,self.refLat),self.boat.position.ycomp()],
            ]
        elif self.courseType == "s":  # Station keeping
            # Define box center relative to boat's position (now offset 50m to the right)
            box_center_x = self.boat.position.xcomp() + meter2degreeX(50, self.refLat)  
            box_center_y = self.boat.position.ycomp()

            # Station keeping outer box with center point offset
            self.waypoints = [
                [box_center_x + meter2degreeX(20,self.refLat), box_center_y + meter2degreeY(20)],
                [box_center_x + meter2degreeX(20,self.refLat), box_center_y - meter2degreeY(20)],
                [box_center_x - meter2degreeX(20,self.refLat), box_center_y - meter2degreeY(20)],
                [box_center_x - meter2degreeX(20,self.refLat), box_center_y + meter2degreeY(20)],
            ]

            # Calculate inner box for station keeping
            self.inner_waypoints = [
                [box_center_x + meter2degreeX(10.0,self.refLat), box_center_y + meter2degreeY(10.0)],
                [box_center_x + meter2degreeX(10.0,self.refLat), box_center_y - meter2degreeY(10.0)],
                [box_center_x - meter2degreeX(10.0,self.refLat), box_center_y - meter2degreeY(10.0)],
                [box_center_x - meter2degreeX(10.0,self.refLat), box_center_y + meter2degreeY(10.0)],
            ]

        # Initialize the autopilot with the actual boat instance
        self.autopilot = Controler(self.boat)
        
        # Call buoy with appropriate arguments based on course type
        if self.courseType == "s":
            self.buoy(self.waypoints, self.inner_waypoints)
        else:
            self.buoy(self.waypoints)

        # Calculate center point for station keeping
        center_x = sum(p[0] for p in self.waypoints) / len(self.waypoints)
        center_y = sum(p[1] for p in self.waypoints) / len(self.waypoints)

        # Inner box waypoints - 10m square for station keeping
        self.inner_waypoints = [
            [center_x+meter2degreeX(10.0,self.refLat), center_y+meter2degreeY(10.0)],
            [center_x+meter2degreeX(10.0,self.refLat), center_y-meter2degreeY(10.0)],
            [center_x-meter2degreeX(10.0,self.refLat), center_y-meter2degreeY(10.0)],
            [center_x-meter2degreeX(10.0,self.refLat), center_y+meter2degreeY(10.0)],
        ]

    def buoy(self, outer_points, inner_points=None):
        """Draw course boundaries, buoys, and circular drift boundary"""
        # Draw outer box corners
        for p in outer_points:
            self.ax.add_patch(plt.Circle(p, meter2degreeY(0.4), color='orange'))
        
        if inner_points:  # Station keeping mode
            # Connect outer box points
            box_lines = outer_points + [outer_points[0]]
            xs, ys = zip(*box_lines)
            self.ax.plot(xs, ys, 'orange')
            
            # Draw inner box corners for reference
            for p in inner_points:
                self.ax.add_patch(plt.Circle(p, meter2degreeY(0.2), color='green'))
            
            # Connect inner box points
            inner_lines = inner_points + [inner_points[0]]
            xs, ys = zip(*inner_lines)
            self.ax.plot(xs, ys, 'green', linestyle='--', linewidth=1)
            
            # Calculate center point
            center_x = sum(p[0] for p in outer_points) / len(outer_points)
            center_y = sum(p[1] for p in outer_points) / len(outer_points)
            
            # Draw circular boundary
            radius_x = abs(inner_points[0][0] - center_x)  # Use inner box half-width as radius
            radius_y = abs(inner_points[0][1] - center_y)  # Use inner box half-height as radius
            circle = patches.Ellipse((center_x, center_y), 2 * radius_x, 2 * radius_y, color='pink', fill=False, linestyle='-',linewidth=2)
            self.ax.add_patch(circle)
            
            # Initialize target point marker
            self.target_point = self.ax.add_patch(plt.Circle((0, 0), meter2degreeY(0.3), color='red', zorder=5))  # Ensure it's drawn on top
            self.target_point.set_visible(False)

    def plotCourse(self, course, color='red'):
        """Plot a course with tracking of created lines"""
        if not course or len(course) < 2:
            return
            
        # Plot course segments
        for i in range(len(course)-1):
            line = self.ax.plot([course[i][0], course[i+1][0]], 
                            [course[i][1], course[i+1][1]], 
                            color=color)[0]
            # Store reference to the line for later removal
            if hasattr(self, 'course_lines'):
                self.course_lines.append(line)

    def createBoat(self):
        self.hullDisplay = []
        self.sailDisplay = []
        self.forceDisplay = []
        self.winches = []
        for i, h in enumerate(self.boat.hulls):
            verts = [(meter2degreeX(p[0]*h.size+ h.position.xcomp(),self.refLat)+self.boat.position.xcomp(),meter2degreeY(p[1]*h.size+h.position.ycomp())+self.boat.position.ycomp()) for p in h.polygon]
            polygon = patches.Polygon(verts, color="gray") 

            r = transforms.Affine2D().rotate_deg_around(self.boat.position.xcomp(),self.boat.position.ycomp(),(self.boat.angle+h.angle).calc())

            polygon.set_transform(r+ self.ax.transData)
            self.hullDisplay.append(self.ax.add_patch(polygon))

            #hull lift
            self.forceDisplay.append(self.ax.plot([0,0],[0,0], color = 'red')[0])
            #hull drag
            self.forceDisplay.append(self.ax.plot([0,0],[0,0], color = 'green')[0])

        for s in self.boat.sails:
            x1 = self.boat.position.xcomp()+meter2degreeX(math.cos((self.boat.angle.calc()+s.position.angle.calc()-90)*math.pi/180)*s.position.norm,self.refLat)
            y1 = self.boat.position.ycomp()+meter2degreeY(math.sin((self.boat.angle.calc()+s.position.angle.calc()-90)*math.pi/180)*s.position.norm)
            x2 = x1+meter2degreeX(math.cos((180+self.boat.angle.calc()+s.angle.calc())*math.pi/180)*s.size,self.refLat)
            y2 = y1+meter2degreeY(math.sin((180+self.boat.angle.calc()+s.angle.calc())*math.pi/180)*s.size)
            self.sailDisplay.append(self.ax.plot([x1,x2],[y1,y2], color = 'yellow')[0])
            #sail lift
            self.forceDisplay.append(self.ax.plot([0,0],[0,0], color = 'gold')[0])
            #sail drag
            self.forceDisplay.append(self.ax.plot([0,0],[0,0], color = 'lime')[0])
            for w in s.winches:
                self.winches.append(self.ax.add_patch(plt.Circle((self.boat.position.xcomp()+meter2degreeX(w.position.xcomp(),self.refLat),self.boat.position.ycomp()+meter2degreeY(w.position.ycomp())),meter2degreeY(w.radius), color='r')))

        #boat net forces:
        self.forceDisplay.append(self.ax.plot([0,0],[0,0], color = 'black')[0])
        #Boat velocity
        self.forceDisplay.append(self.ax.plot([0,0],[0,0], color = 'magenta')[0])

        # Add wind direction arrow
        # Start with arbitrary position, we'll update it in update method
        center_x = (self.ax.get_xlim()[0] + self.ax.get_xlim()[1]) / 2
        center_y = (self.ax.get_ylim()[0] + self.ax.get_ylim()[1]) / 2
        arrow_length = (self.ax.get_xlim()[1] - self.ax.get_xlim()[0]) * 0.1  # 10% of plot width
        self.wind_arrow = self.ax.quiver(center_x, center_y, 0, 0, 
                                    color='black', scale=2, scale_units='xy',
                                    width=0.005, headwidth=4, headlength=5)

    def update(self, auto, showForces):
        for i in range(numCycle):
            self.boat.update(1/fps)
            if auto:
                self.autopilot.update(1/fps)

        forceScale = 0.01

        # Update target point visualization if in station keeping mode
        if (auto and 
            hasattr(self.autopilot, 'control_mode') and 
            self.autopilot.control_mode == "station_keeping" and 
            self.autopilot.station_keeper is not None and  # Add explicit check
            hasattr(self.autopilot.station_keeper, 'upwind_target')):  # Check for attribute
            
            target = self.autopilot.station_keeper.upwind_target
            if self.target_point:  # Check if target point exists
                self.target_point.center = (target[0], target[1])
                self.target_point.set_visible(True)
        elif hasattr(self.target_point, 'set_visible'):
            self.target_point.set_visible(False)

        #hulls
        for i, h in enumerate(self.hullDisplay):
            hull = copy.deepcopy(self.boat.hulls[i])
            hull.position.angle += Angle.norm(self.boat.angle)

            cx = self.boat.position.xcomp() + meter2degreeX(hull.position.xcomp(),self.refLat)
            cy = self.boat.position.ycomp() + meter2degreeY(hull.position.ycomp())

            r = transforms.Affine2D().rotate_deg_around(cx,cy,(self.boat.angle+self.boat.hulls[i].angle).calc())
            sum = r + self.ax.transData

            if showForces:
                self.forceDisplay[2*i].set_linestyle("solid")
                self.forceDisplay[2*i+1].set_linestyle("solid")
                #lift
                self.forceDisplay[2*i].set_xdata([cx,cx+forceScale*meter2degreeX(self.boat.hullLiftForceandMoment(i)[0].xcomp(),self.refLat)])
                self.forceDisplay[2*i].set_ydata([cy,cy+forceScale*meter2degreeY(self.boat.hullLiftForceandMoment(i)[0].ycomp())])
                #drag
                self.forceDisplay[2*i+1].set_xdata([cx,cx+forceScale*meter2degreeX(self.boat.hullDragForceandMoment(i)[0].xcomp(),self.refLat)])
                self.forceDisplay[2*i+1].set_ydata([cy,cy+forceScale*meter2degreeY(self.boat.hullDragForceandMoment(i)[0].ycomp())])
            else:
                self.forceDisplay[2*i].set_linestyle("None")
                self.forceDisplay[2*i+1].set_linestyle("None")
            
            verts = [(meter2degreeX(p[0]*self.boat.hulls[i].size,self.refLat)+cx,meter2degreeY(p[1]*self.boat.hulls[i].size)+cy) for p in self.boat.hulls[i].polygon]
            self.hullDisplay[i].set_xy(verts)
            self.hullDisplay[i].set_transform(sum)

            # Only try to update target point if we're in station keeping mode AND all required objects exist
            if (auto and 
                hasattr(self.autopilot, 'control_mode') and 
                self.autopilot.control_mode == "station_keeping" and 
                self.autopilot.station_keeper is not None and
                hasattr(self.target_point, 'center')):
                target = self.autopilot.station_keeper.upwind_target
                self.target_point.center = (target[0], target[1])
                self.target_point.set_visible(True)
            elif hasattr(self.target_point, 'set_visible'):
                self.target_point.set_visible(False)

        #sails
        for i, s in enumerate(self.sailDisplay):
            x1 = self.boat.position.xcomp()+meter2degreeX(math.cos((self.boat.angle.calc()+self.boat.sails[i].position.angle.calc()-90)*math.pi/180)*self.boat.sails[i].position.norm,self.refLat)
            y1 = self.boat.position.ycomp()+meter2degreeY(math.sin((self.boat.angle.calc()+self.boat.sails[i].position.angle.calc()-90)*math.pi/180)*self.boat.sails[i].position.norm)
            x2 = x1+meter2degreeX(math.cos((180+self.boat.angle.calc()+self.boat.sails[i].angle.calc())*math.pi/180)*self.boat.sails[i].size,self.refLat)
            y2 = y1+meter2degreeY(math.sin((180+self.boat.angle.calc()+self.boat.sails[i].angle.calc())*math.pi/180)*self.boat.sails[i].size)
            
            CEx = x1+meter2degreeX(math.cos((180+self.boat.angle.calc()+self.boat.sails[i].angle.calc())*math.pi/180)*self.boat.sails[i].size/2,self.refLat)
            CEy = y1+meter2degreeY(math.sin((180+self.boat.angle.calc()+self.boat.sails[i].angle.calc())*math.pi/180)*self.boat.sails[i].size/2)

            if showForces:
                #lift
                self.forceDisplay[2*i+len(self.hullDisplay)*2].set_linestyle("solid")
                self.forceDisplay[2*i+1+len(self.hullDisplay)*2].set_linestyle("solid")
                self.forceDisplay[2*i+len(self.hullDisplay)*2].set_xdata([CEx,CEx+forceScale*meter2degreeX(self.boat.sailLiftForce(i).xcomp(),self.refLat)])
                self.forceDisplay[2*i+len(self.hullDisplay)*2].set_ydata([CEy,CEy+forceScale*meter2degreeY(self.boat.sailLiftForce(i).ycomp())])
                #drag
                self.forceDisplay[2*i+1+len(self.hullDisplay)*2].set_xdata([CEx,CEx+forceScale*meter2degreeX(self.boat.sailDragForce(i).xcomp(),self.refLat)])
                self.forceDisplay[2*i+1+len(self.hullDisplay)*2].set_ydata([CEy,CEy+forceScale*meter2degreeY(self.boat.sailDragForce(i).ycomp())])
            else:
                self.forceDisplay[2*i+len(self.hullDisplay)*2].set_linestyle("None")
                self.forceDisplay[2*i+1+len(self.hullDisplay)*2].set_linestyle("None")

            s.set_xdata([x1,x2])
            s.set_ydata([y1,y2])
            for idx, w in enumerate(self.boat.sails[i].winches):
                pos = w.position.meter2degree(self.refLat)
                pos.angle += self.boat.angle - Angle(1,90)
                pos += self.boat.position
                self.winches[idx].set(center =(pos.xcomp(),pos.ycomp()))

        if showForces:
            #boat net forces
            f = self.boat.forces["sails"]+self.boat.forces["hulls"]
            self.forceDisplay[-2].set_xdata([self.boat.position.xcomp(),self.boat.position.xcomp()+forceScale*meter2degreeX(f.xcomp(),self.refLat)])
            self.forceDisplay[-2].set_ydata([self.boat.position.ycomp(),self.boat.position.ycomp()+forceScale*meter2degreeY(f.ycomp())])
            #boat velocity
            self.forceDisplay[-1].set_xdata([self.boat.position.xcomp(),self.boat.position.xcomp()+forceScale*meter2degreeX(self.boat.linearVelocity.xcomp(),self.refLat)])
            self.forceDisplay[-1].set_ydata([self.boat.position.ycomp(),self.boat.position.ycomp()+forceScale*meter2degreeY(self.boat.linearVelocity.ycomp())])
        else:
            self.forceDisplay[-2].set_linestyle("None")
            self.forceDisplay[-1].set_linestyle("None")

        # Update wind arrow position and direction
        # Position arrow in top-right corner of visible area
        visible_x = self.ax.get_xlim()
        visible_y = self.ax.get_ylim()
        arrow_x = visible_x[0] + (visible_x[1] - visible_x[0]) * 0.85  # 85% across
        arrow_y = visible_y[0] + (visible_y[1] - visible_y[0]) * 0.85  # 85% up
        
        # Calculate arrow components based on wind angle
        wind_angle_rad = math.radians(self.boat.wind.angle.calc())
        arrow_length = (visible_x[1] - visible_x[0]) * 0.1  # 10% of plot width
        dx = arrow_length * math.cos(wind_angle_rad)
        dy = arrow_length * math.sin(wind_angle_rad)
        
        # Update arrow
        self.wind_arrow.set_UVC(dx, dy)
        self.wind_arrow.set_offsets([arrow_x, arrow_y])

class display:
    def __init__(self,location,boat):
        self.f, self.axes = plt.subplot_mosaic('AAABD;AAACC', figsize=(12, 5))
        self.pause = False
        self.track = False
        self.auto = False
        self.forceShow = True
        self.time = 0
        self.course_lines = []  # Track active course lines

        self.boat = boatDisplayShell(boat, self.axes['A'], boat.position.ycomp())
        self.map(location)
        self.boat.initAuto()
        self.boat.createBoat()

        self.boat.course_lines = []

        self.axes['B'].set_title('Display Settings')
        self.axes['B'].axis('off')
        self.displaySettings()

        self.axes['C'].set_title('Debug Values')
        self.axes['C'].axis('off')
        self.text = []
        self.text.append(self.axes['C'].text(0, 0.9, "Boat LVelocity V:0", fontsize=9))
        self.text.append(self.axes['C'].text(0, 0.8, "Boat AVelocity V:0", fontsize=9))
        self.text.append(self.axes['C'].text(0, 0.7, "Hull Apparent A:0", fontsize=9))
        self.text.append(self.axes['C'].text(0, 0.6, "Sail Apparent A:0", fontsize=9))
        self.text.append(self.axes['C'].text(0, 0.5, "Hull lift F:0", fontsize=9))
        self.text.append(self.axes['C'].text(0, 0.4, "Hull Drag F:0", fontsize=9))
        self.text.append(self.axes['C'].text(0, 0.3, "Sail lift F:0", fontsize=9))
        self.text.append(self.axes['C'].text(0, 0.2, "Sail Drag F:0", fontsize=9))
        self.text.append(self.axes['C'].text(0, 0.1, "Boat Position V:0", fontsize=9))
        self.text.append(self.axes['C'].text(0, 0.0, "Time (s):0", fontsize=9))
        # Add station keeping state text element
        self.text.append(self.axes['C'].text(0, -0.1, "Station Keeping State: Inactive", fontsize=9))
        
        self.displayValues()

        self.axes['D'].set_title('Boat Controls')
        self.axes['D'].axis('off')
        self.boatControls()

        self.course_lines = []  # Track active course lines

        #credits
        plt.figtext(0, 0.01, 'Map: © OpenStreetMap contributors', fontsize = 10)
    
    def clear_paths(self):
        """Clear all path visualization lines"""
        # Remove stored course lines
        if hasattr(self.boat, 'course_lines'):
            for line in self.boat.course_lines:
                line.remove()
            self.boat.course_lines = []
            
        # Double check for any remaining course lines
        lines_to_remove = []
        for line in self.axes['A'].get_lines():
            if line.get_color() in ['red', 'green']:
                lines_to_remove.append(line)
                
        # Remove the lines after identifying them
        for line in lines_to_remove:
            line.remove()

    def bUpdate(self,v):
        self.boat.boat.hulls[-1].angle = Angle(1,self.bRot.val)

    def sUpdate(self,v):
        if len(self.boat.boat.sails[0].winches) == 0:
            self.boat.boat.sails[0].angle = Angle(1,self.sRot.val)
        else:
            if self.sRot.val != self.boat.boat.sails[0].winches[0].rot.calc():
                self.boat.boat.sails[0].setSailRotation(Angle(1,self.sRot.val))

    def wUpdate(self, v):
        """Handle wind angle updates from slider"""
        try:
            # Update wind angle
            self.boat.boat.wind.angle = Angle(1, self.wRot.val + 180)
            
            # Only recalculate if in autopilot mode
            if self.auto and hasattr(self.boat, 'autopilot') and self.boat.autopilot:
                # Try to handle wind change with new control system
                if hasattr(self.boat.autopilot, 'handle_wind_change'):
                    if self.boat.autopilot.handle_wind_change():
                        # Clear existing paths
                        self.clear_paths()
                        # Plot new course if we have an active course
                        if hasattr(self.boat.autopilot, 'active_course') and self.boat.autopilot.active_course:
                            self.boat.plotCourse(self.boat.autopilot.active_course, 'green')
        except Exception as e:
            print(f"Error updating wind direction: {str(e)}")
            # Optionally reset to previous state or take other recovery action; not currently necessary
    
    def spUpdate(self,v):
        global numCycle
        numCycle = int(v)

    def boatControls(self):
        bax = plt.axes([0, 0, 1, 1])
        bforcesInp = InsetPosition(self.axes['D'], [0.6, 0.9, 0.9, 0.1])
        bax.set_axes_locator(bforcesInp)
        self.bRot = Slider(
            ax=bax,
            label="Rudder Rotation:",
            valmin=-20,
            valmax=20,
            valinit=self.boat.boat.angle.calc(),
        )
        self.bRot.on_changed(self.bUpdate)
        
        sax = plt.axes([0, 0, 1, 1])
        sforcesInp = InsetPosition(self.axes['D'], [0.45, 0.75, 0.9, 0.1])
        sax.set_axes_locator(sforcesInp)
        self.sRot = Slider(
            ax=sax,
            label="Sail Rotation:",
            valmin=-90,
            valmax=90,
            valinit=self.boat.boat.sails[0].angle.calc(),
        )
        self.sRot.on_changed(self.sUpdate)

        wax = plt.axes([0, 0, 1, 1])
        wforcesInp = InsetPosition(self.axes['D'], [0.45, 0.60, 0.9, 0.1])
        wax.set_axes_locator(wforcesInp)
        self.wRot = Slider(
            ax=wax,
            label="Wind Angle:",
            valmin=0,
            valmax=360,
            valinit=Angle.norm(self.boat.boat.wind.angle+Angle(1,180)).calc(),
        )
        self.wRot.on_changed(self.wUpdate)

        spax = plt.axes([0, 0, 1, 1])
        spforcesInp = InsetPosition(self.axes['D'], [0.45, 0.45, 0.9, 0.1])
        spax.set_axes_locator(spforcesInp)
        self.spRot = Slider(
            ax=spax,
            label="Comp Speed:",
            valmin=1,
            valmax=50,
            valinit=1,
        )
        self.spRot.on_changed(self.spUpdate)

    def pauseT(self,t):
        self.pause = not self.pause
        if self.pause:
            self.pauseButton.label.set_text('Play Animation')
        else:
            self.pauseButton.label.set_text('Pause Animation')

    def trackZ(self,t):
        self.track = not self.track
        if self.track:
            self.zoomButton.label.set_text('Stop Tracking')
        else:
            self.zoomButton.label.set_text('Track Boat')
            
    def autoF(self,t):
        self.auto = not self.auto
        if self.auto:
            # Set up the display reference in the controller
            self.boat.autopilot.display = self
            
            # Initialize the course with existing waypoints
            initial_course = self.boat.autopilot.plan(self.boat.courseType, self.boat.waypoints)
            
            if self.boat.courseType == "s":  # Station keeping
                print("Initializing station keeping...")
                self.boat.autopilot.control_mode = "station_keeping"
                if not hasattr(self.boat.autopilot, 'station_keeper'):
                    self.boat.autopilot.station_keeper = StationKeepingController(
                        self.boat.boat,  # Pass the actual boat instance
                        self.boat.waypoints,
                        self.boat.autopilot,
                        self.clear_paths
                    )
            else:
                self.clear_paths()
                self.boat.plotCourse(initial_course, 'green')
                
            self.autoButton.label.set_text('Auto Pilot: ON')
        else:
            self.clear_paths()
            self.autoButton.label.set_text('Auto Pilot: OFF')
                
    def forceS(self,t):
        self.forceShow = not self.forceShow
        if self.forceShow:
            self.forceButton.label.set_text('Hide Forces')
        else:
            self.forceButton.label.set_text('Show Forces')
            
    def displaySettings(self):
        F_button_ax = plt.axes([0, 0, 1, 1])
        forcesInp = InsetPosition(self.axes['B'], [0, 0.9, 0.9, 0.1])
        F_button_ax.set_axes_locator(forcesInp)
        self.forceButton = Button(F_button_ax, 'Hide Forces')
        self.forceButton.on_clicked(self.forceS)

        P_button_ax = plt.axes([0, 0, 1, 1])
        pauseInp = InsetPosition(self.axes['B'], [0, 0.78, 0.9, 0.1])
        P_button_ax.set_axes_locator(pauseInp)
        self.pauseButton = Button(P_button_ax, 'Pause Animation')
        self.pauseButton.on_clicked(self.pauseT)

        Z_button_ax = plt.axes([0, 0, 1, 1])
        zoomInp = InsetPosition(self.axes['B'], [0, 0.66, 0.9, 0.1])
        Z_button_ax.set_axes_locator(zoomInp)
        self.zoomButton = Button(Z_button_ax, 'Track Boat')
        self.zoomButton.on_clicked(self.trackZ)

        A_button_ax = plt.axes([0, 0, 1, 1])
        autoInp = InsetPosition(self.axes['B'], [0, 0.54, 0.9, 0.1])
        A_button_ax.set_axes_locator(autoInp)
        self.autoButton = Button(A_button_ax, 'Auto Pilot: OFF')
        self.autoButton.on_clicked(self.autoF)

    def displayValues(self):
        #Velocity
        self.text[0].set_text("Boat LVelocity V:" + rm_ansi(str(self.boat.boat.linearVelocity)))
        self.text[1].set_text("Boat AVelocity V:" + rm_ansi(str(round(self.boat.boat.rotationalVelocity*180/math.pi*10000)/10000)))
        #Apparent Wind
        self.text[2].set_text("Hull Apparent V:" + rm_ansi(str(self.boat.boat.hullAparentWind(1))).replace("Vector",""))
        self.text[3].set_text("Sail Apparent V:" + rm_ansi(str(self.boat.boat.sailAparentWind(0))).replace("Vector",""))
        #Hull Forces
        self.text[4].set_text("Hull lift F:" + rm_ansi(str(self.boat.boat.hullLiftForceandMoment(1)[0])))
        self.text[5].set_text("Hull Drag F:" + rm_ansi(str(self.boat.boat.hullDragForceandMoment(1)[0])))
        #Sail Forces
        self.text[6].set_text("Sail lift F:" + rm_ansi(str(self.boat.boat.sailLiftForce(0))))
        self.text[7].set_text("Sail Drag F:" + rm_ansi(str(self.boat.boat.sailDragForce(0))))
        #pos 
        xs = str(self.boat.boat.position.xcomp())
        ys = str(self.boat.boat.position.ycomp())
        self.text[8].set_text("Boat Position V:" + rm_ansi("("+xs[0:min(10,len(xs))]+", "+ys[0:min(10,len(ys))]+")"))
        self.text[9].set_text("Time (s):"+str(self.time))

        # Update station keeping state if active
        if (hasattr(self.boat.autopilot, 'control_mode') and 
            self.boat.autopilot.control_mode == "station_keeping" and 
            self.boat.autopilot.station_keeper is not None):
            state = self.boat.autopilot.station_keeper.state
            self.text[10].set_text(f"Station Keeping State: {state}")
        else:
            self.text[10].set_text("Station Keeping State: Inactive")

    def map(self,location):
        cords = regionPolygon(location)
        x = [i[0] for i in cords]
        y = [i[1] for i in cords]

        mx = min(x)
        my = min(y)

        x = [(i-mx) for i in x]
        y = [(i-my) for i in y]

        bx = (self.boat.boat.position.xcomp()-mx)
        by = (self.boat.boat.position.ycomp()-my)
        self.boat.boat.setPos(Vector(Angle(1,round(math.atan2(by,bx)*180/math.pi*10000)/10000),math.sqrt(bx**2+by**2)))

        self.axes['A'].set_title(location.split(" ")[0]+ " map")
        self.axes['A'].fill(x,y,'b')
        self.axes['A'].axis([min(x), max(x),min(y), max(y)])
        self.axes['A'].grid()
    
    def updateCycle(self, f):
        if not self.pause:
            # Update boat state
            self.boat.update(self.auto, self.forceShow)
            self.time += 1/fps * numCycle
            
            # Update control displays
            self.bRot.set_val(printA(self.boat.boat.hulls[-1].angle.calc()))
            
            if len(self.boat.boat.sails[0].winches) == 0:
                self.sRot.set_val(printA(self.boat.boat.sails[0].angle.calc()))
            else:
                self.sRot.set_val(printA(self.boat.boat.sails[0].winches[0].rot.calc()))
            
            # Handle station keeping visualization
            if (self.auto and 
                hasattr(self.boat.autopilot, 'control_mode') and 
                self.boat.autopilot.control_mode == "station_keeping" and
                self.boat.autopilot.station_keeper is not None):
                
                station_keeper = self.boat.autopilot.station_keeper
                
                # Only draw path if we're not in DRIFTING state
                if (hasattr(station_keeper, 'state') and 
                    station_keeper.state != "DRIFTING" and
                    hasattr(station_keeper, 'target_path') and 
                    station_keeper.target_path):
                    self.clear_paths()
                    self.boat.plotCourse(station_keeper.target_path, 'red')  # Call plotCourse through boat instance
            
            # Handle normal course updates
            elif self.auto and hasattr(self.boat.autopilot, 'target_reached'):
                if self.boat.autopilot.target_reached():
                    # Clear and redraw path when target is reached
                    self.clear_paths()
                    self.boat.plotCourse(self.boat.autopilot.active_course, 'green')  # Call plotCourse through boat instance
            
            if self.track:
                dx = self.axes['A'].get_xlim()[1]-self.axes['A'].get_xlim()[0]
                dy = self.axes['A'].get_ylim()[1]-self.axes['A'].get_ylim()[0]
                dm = max(dx,dy)
                self.axes['A'].set_xlim(self.boat.boat.position.xcomp()-dm/2,
                                      self.boat.boat.position.xcomp()+dm/2)
                self.axes['A'].set_ylim(self.boat.boat.position.ycomp()-dm/2,
                                      self.boat.boat.position.ycomp()+dm/2)
            
            self.displayValues()

    def runAnimation(self):
        anim = FuncAnimation(self.f, self.updateCycle, interval=1000/fps,cache_frame_data=False)
        plt.show()

if __name__ == "__main__":
    lakeAttitash = "Lake Attitash, Amesbury, Essex County, Massachusetts, USA"
    lakeShoreline = "Shoreline lake, Mountain View, Santa Clara County, California, United States"

    # Define the data directory path relative to the current script location
    data_dir = os.path.join(os.path.dirname(__file__), "data")

    vaka = foil(os.path.join(data_dir, "xf-naca001034-il-1000000-Ex.csv"), 997.77, 0.521, rotInertia=2.69, size=1.8)
    ama1 = foil(os.path.join(data_dir, "naca0009-R0.69e6-F180.csv"), 997.77, 0.1768, position=Vector(Angle(1,90), 0.6), rotInertia=3.939, size=1.5)
    ama2 = foil(os.path.join(data_dir, "naca0009-R0.69e6-F180.csv"), 997.77, 0.1768, position=Vector(Angle(1,-90), 0.6), rotInertia=3.939, size=1.5)
    rudder = foil(os.path.join(data_dir, "naca0015-R7e7-F180.csv"), 997.77, 0.0588, position=Vector(Angle(1,180), vaka.size/2), rotInertia=0.01, size=0.3)

    offset = 0.45
    BabordWinch = Winch(Vector(Angle(1,180), 0.6) + Vector(Angle(1,270), offset), 30, 0.025)
    TribordWinch = Winch(Vector(Angle(1,0), 0.6) + Vector(Angle(1,270), offset), 30, 0.025)

    sail = foil(os.path.join(data_dir, "MarchajSail.cvs"), 1.204, 2.03, position=Vector(Angle(1,90), 0.4), rotInertia=0, size=0.7, winches=[BabordWinch, TribordWinch])

    sail.setSailRotation(Angle(1,0))
    wind = Vector(Angle(1,270),5.3) # Going South wind, 7 kn
    xpos = -122.09064
    ypos = 37.431749
    boat = Boat([ama1,vaka,ama2,rudder],[sail],wind,mass=15,refLat=ypos)
    boat.angle = Angle(1,-93)
    sail.angle = Angle(1,75)
    sail.setSailRotation(sail.angle)
    boat.setPos(Vector(Angle(1,round(math.atan2(ypos,xpos)*180/math.pi*10000)/10000),math.sqrt(xpos**2+ypos**2)))
    polars = input("recalc Polars Y/N:\n")
    if "y" in polars.lower():
        generatePolars(boat,"MarPol")
    render = display(lakeShoreline,boat)
    render.runAnimation()