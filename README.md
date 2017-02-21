## What is this repository for? ##
For regulating commands to the hardware API, choosing between manual and automatic

Script for creating catkin ws and downloading repos: https://drive.google.com/open?id=0BxRJZY1j9wVVMDBwVDBBRG12cTA

# How to run the truck #

## 0. You will need: ##
* The truck
* a box to place it on, elevating the back wheels, so it doesn't drive off when starting
* Hdmi cable
* Charged batteries
* A powerbank to power the pi while driving.
* Xbox or PS3 controller
* Laptop with ROS installed
* ackermann_msgs and joy packages: 

    `sudo apt-get update`

    `sudo apt-get install ros-kinetic-ackermann-msgs`

    `sudo apt-get install ros-kinetic-joy`


* To have given Thomas the MAC-address on your computer (not sure if this is actually needed?)
* Connect to vnet on the truck and the laptop

## 1. The truck ##
   * Boot the Raspberry PI with HDMI plugged in. (Don't forget to turn on projector)
   * Open two terminal windows and run `roscore` and in the other `rosrun truck_hw_api cmd_node.py` (can be done either through ssh or directly, but i recommend directly)
   
## 2. The camera-system: ##
 
`ssh 192.168.5.31 -X -l bachelor`


 (Password is six characters, similar to the vnet password)
   

`cd repository/visionlocalization_old/build`


`./script.sh`

   
## 3. The laptop ##
   * Plugin the xbox controller.
      - If you wanna use the ps3 controller, try this: [http://askubuntu.com/questions/409761/how-do-i-use-a-ps3-sixaxis-controller-with-ubuntu-to-control-games](http://askubuntu.com/questions/409761/how-do-i-use-a-ps3-sixaxis-controller-with-ubuntu-to-control-games)
   * `export ROS_HOSTNAME=YOUR_IP` `export ROS_MASTER_URI=http://TRUCK_IP:11311` with correct IP addresses. (check ip with ifconfig) (You need to export this in every new terminal window)
* IMPORTANT: Before running the next command, make sure the back wheels are elevated so that it doesn't drive off.
* Run 


`roslaunch truck_master master.launch hw_api:=0`


 (this launches all nodes except the hardware API)


      - If you are using the ps3 controller, you need to add gamepad_type:="ps3" to the launch command
   
## 4. How to drive: ##
   - ![Controls.png](https://bitbucket.org/repo/nqxL85/images/3204438201-Untitled.png)
   - The Dead Man's Switch needs to be pressed for the truck to move. (Applies to both manual and automatic driving)
   - The select button changes the behavior of the gas pedal (nothing else), if it is used for reversing or driving forwards.

## 5. How to kill a node without killing all nodes: ##
    

`rosnode kill *nodename*`

* Change what you want to change in the source-code and start it again:

`rosrun package_name node.py`
 
* Or run the python file directly.

`cd /path/to/scripts`
`python node.py`
## 5. Troubleshooting:  
   *  If the PI turns off randomly while driving, it means there's an issue with the power. Either the batteries have ran out, or the batteries aren't strong enough to power both the     motor and the pi.
       - Charge batteries
       - Use a powerbank for the PI to solve this problem.
             - Turn off battery, plugin powerbank, boot PI (dont forget HDMI), wait a few moments, then turn on the battery again.
   * What is the password?
      - Ask in Slack
   * Nothing happens when pressing the controller?
       - This might be a variety of issues, 
       - echo rostopics to see where the issue is.
           - rostopic echo joy
           - rostopic echo man_drive
           - rostopic echo dead_mans_switch
           - rostopic echo master_drive
       - double check "export ROS_***" commands above
       - Check if the controller you are truing to use are really /dev/input/js0. If it isn't (it might be js1 for example), 
         then you need to start the joy_node with the argument "dev:="/dev/input/js1""
         - To do so add this line:
             <param name="dev" type="string" value="/dev/input/js1" />
            in the node-tag for the joy_node in master.launch
   - no data is being published on "error"
      - make sure there is a visible tag ;)
   - I get a broken pipe error
      - start the roscore and hw_api from the truck instead of through ssh