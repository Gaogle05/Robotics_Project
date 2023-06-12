# NOTE: "src" folder contains latest code by Salmaan

# Robotics_Project
SurveillanceBot

gmapping tutorial: http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Make%20a%20map%20and%20navigate%20with%20it

Things to do:
- Explore & Build Map using gmapping 
- Have a node to read (x,y) location from std input, and then navigate to that point. (Use gazebo state service) - *Should be able to navigate to at least 2 points
- Have a node that scans pixels visually to fing "Utility Cart" (Bright green) - Output "Yes" for found, otherwise "No"
- Video to show everything works.

# To run the code for intruderAlert
## Note you need to move this into the correct folder where the other code is (I didn't want to break things so I didn't add it directly into your folder.

New Tab
``` source devel/setup.bash ```
``` ./startWorld ```

New Tab
``` source devel/setup.bash ```
``` roslaunch turtlebot_rviz_launchers view_robot.launch ```

In rviz
- Under the display panel:
    - Choose point cloud 
    - Choose Image
        - Under Image
        - Change Image Topic to /camera/rgb/image_raw
        
 In Gazebo from the insert tab insert Utility Cart
    - Place the cart in desired position 
 
 
 To run intruderAlert 
 - New terminal tab:
    ``` source devel/setup.bash ```
    ``` rostopic echo intruderAlert ```
 
 
