**This is the third iteration of the Kehillah Sailbot boat simulation program for the 2024-2025 Sailbot competition**\

*This is a simulation program to allow for the creation of a navigation program, I don't expect this to run on the final boat.*
# Main Goal:
* Simulate aero and hydro dynamic forces on boat sails and hulls
* Allow for navigation algorithm testing that accurately replicates how the boat would behave in water
* Calculate optimal sail trim, Best VMG, and general headings for variable hulls and sails

# TODO:
* Sail physics
    * Sail should behave like real sail to properly simulate jibes and tacks
    * Fix up variable system, display variables are no longer in use and the data angles are being problamatic

# Citations:
* Boeing, G. 2017. ["OSMnx: New Methods for Acquiring, Constructing, Analyzing, and Visualizing Complex Street Networks."](https://geoffboeing.com/publications/osmnx-complex-street-networks/) Computers, Environment and Urban Systems 65, 126-139. doi:10.1016/j.compenvurbsys.2017.05.004

# Notes:
* As I switched from pyglet to matplotlib for display the `display` type for variables shouldn't be used

# Conventions/Units:
* Wind and flow stuff is a vector that points in the direction of the flow
* For apparent angles if the wind has a lower angle than the boat it's negative angles
*Due to the complexity of the problem multiple angle, positioning, speed, force, ... systems are inevitable*
* ## Overall
    * ### Currently there seems to be 3 main "types" of units: Data, Calculations, Display.
    _____________________
    * **Data variables** are usually external data that are nessesary to the boat's function (wind tunel data, speed polars, ect).
        * They have very specific formats that make sense for their application (ei: upwards is 0&deg;).
    _____________________
    * **Calculations variables** are the main types of variables and make sense for the big picture, these are the "conceptual" units.
        * Typically uses unit circle angles, cardinal directions (the exact types we are using are documented further down).
        * These are usualy used in the math part.
        * Generally all the calculations variables are in metric, that is forces in Newtons, distance in Meters, air density in kg/m^3.
    _____________________
    * Finally **display variables**, these are purely for the graphics part of the sim, they might not be included in the actuall boat's code.
        * These can be the pygame cordinate system, pyglet, ect.
    _____________________
    * ### To avoid confusion all important variables will be instances of a class "[`Variable`](#variable-class)"
* ## Angles
    * Always in degrees unless there is a good reason
    * Pyglet angles (do not loop, can be negative)
        * The angle in the simulator is the angle the wind is coming from, 
            * e.g. Wind angle = 0 degrees, means wind is blowing from East -> West
    ```
           |90
         0---180 
           |270
    ```
    * data angles (no negatives, 0-180)
    ```
           |0
     (-)90--- 90
           |180
    ```
    * Calculation angles (unit circle):
    ```
           |90
       180---0
           |270
    ```
* ## Positions
    * Pyglet Cordinates (Display and Calculation) in meters
    ```
        Bottom left corner is (0,0), y increases from there
        ^
        |
      (0,0)--->
    ```
    * Eventually GPS lat and long.


# Variable Class
* ### Has 3 main properties, data, calc, disp and the nessesary framework for seemless interaction between the various types
* Variable will be base class that will have implementations such as `Angle`, `Vector`.
    * The `Vector` class might be used to represent: Forces, positions, speeds, ect
* Constructor will take type of variable and value.
* Something like a coeffeciant isn't being currently made into a variable
* Format: `Variable(type,value)`:
    * Types are numbers 0-2, 0 is data variable, 1 is calc variable, 2 is display variable.
* `.data()`, `.calc()`, `.disp()` are getter functions for numerical values (these are not made to be used in calculation code but only within the variable class).
* Magic methods for addition, will update all variable types.
    * The main utitility of the class will be the ability to add, multiply, convert and use different unit systems seemlessly through magic methods.
    * Ei: you have a data angle `da` and a calc angle `ca`, you can just create a new angle `k=da+ca` that behaves correctly.

# Foil Class
* ### The main goal is to keep track of the various foils and calculate the forces acting on them given a wind source.
* ### Construtor:
    * Format: `Foil(datasheet, material, wetted area)`.
        * Datasheet: String of location of datasheet 
        * Material: Density of the material the foil comes in **CONTACT** to, that is water, or air, in kg/m^3.
        * Wetted area: Area which is in contact with the external air/waterflow in m^2.
* ### Lift and Drag:
    * Format: `foil.lift(Velocity)` or `foil.drag(Velocity)`.
        * Velocity: Velocity `Vector` generated by wind/aparent wind or boat against water. (m/s)
    * Returns lift or drag force, in Newtons, remember lift is perpendicular to AIR**FLOW**, and drag in line with the AIR**FLOW**.
* ### Lift and Drag forces:
    * Format: `foil.liftForce(Velocity)` or `foil.dragForce(Velocity)`.
        * Velocity: Velocity `Vector` generated by wind/aparent wind or boat against water. (m/s)
    * Returns lift and drag force `Vector` relative to foil

# Boat Class
* ### Boat holds various foils together, manages position and crucially: velocity, wind, aparent wind/water for foils, and forces
* ### Construtor:
    * Format: `Boat(hulls, sails, wind, position = Vector(Angle(1,0),0))`
        * Hulls: Array of hydrofoils
        * Sails: Array of sails
        * Wind: Single wind `Vector` (vector represents wind so angles must be adjusted)
        * Position: relative position to parent object in `Vector` format.
* ### Global Aparent Wind:
    * Format: `boat.globalAparentWind()`
    * Returns `Vector` of the aparent wind in terms of global positioning
* ### Sail Aparent Wind:
    * Format: `boat.sailAparentWind(idx=0)`
        * Where `idx` is sail index, defaults to 0
    * Returns `Vector` of the local aparent wind in terms of given sail
* ### Sail and Hull Drag Force:
    * Format: `boat.sailDragForce(idx=0)` or `boat.hullDragForce(idx=0)`
        * Where `idx` is sail or hull index, defaults to 0
    * Returns Force `Vector` of the global drag of a given child foil in terms of given sail or hull
* ### Sail and Hull Lift Force:
    * Format: `boat.sailLiftForce(idx=0)` or `boat.hullLiftForce(idx=0)`
        * Where `idx` is sail or hull index, defaults to 0
    * Returns Force `Vector` of the global lift of a given child foil in terms of given sail or hull
* ### Force update functions:
    * Format: `boat.updateSailForces()` or `boat.updateHullForces()`
    * Returns void, updates `self.forces["sails"]` or `self.forces["hulls"]` respectivly.
