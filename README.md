# NOTE: "src" folder contains latest code by Salmaan

# Robotics_Project
SurveillanceBot

gmapping tutorial: http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Make%20a%20map%20and%20navigate%20with%20it

Things to do:
- Explore & Build Map using gmapping 
- Have a node to read (x,y) location from std input, and then navigate to that point. (Use gazebo state service) - *Should be able to navigate to at least 2 points
- Have a node that scans pixels visually to fing "Utility Cart" (Bright green) - Output "Yes" for found, otherwise "No"
- Video to show everything works.

Link to Report: https://www.overleaf.com/1179483169vfwvkpqbxhtm

# map origin: [-13.80, 12.20, 0.00]
# map resolution/scale: 0.05

# convert world coords to pixel coords and vice versa
def world_to_pixel(x1, y1):
    x = round((13.80 - x1)/0.50)
    y = round((12.20 - y1)/0.50)
    return x, y

def pixel_to_world(x1, y1):
    x = 13.80 - (x1 * 0.05)
    y = 12.20 - (y1 * 0.05)
    return x, y
