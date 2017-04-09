## What is this repository for? ##
For regulating commands to the hardware API, choosing between manual and automatic. Also a central hub for the truck project

## Installation ##

* Download this [script](https://drive.google.com/open?id=0BxRJZY1j9wVVQUFOTUxBc2ZZaXc), which downloads ROS, creates a new catkin workspace and clones down all the repositories you need. If you don't need to do all this, just comment out the lines you don't want to run

    * Put the file where you want to create the new workspace and run it
     
          `cd path/to/trucker`      

          `./trucker`


# How to run the simulator
* `roslaunch truck_master master.launch`
* Use the "2D Nav Goal" tool to set the goal.
    * Watch as the pathplanning does it's thing and the truck starts to move :)
* To set the position and direction of the truck, use the "2D Pose Estimate" tool
* To put subgoals use the "Publish Position" tool
* Add obstacles by focusing the window with the pyplot and press the number of the obstacle you want to add. (on the keyboard)
    * Watch as the rviz map is updated :)
* If the pathplanning seems to be stuck in an infinite loop you may have to kill the node (or just restart everything).
    * `rosnode kill path_planning`
    * Remove all obstacles
    * `rosrun path_planning path_planning_node.py`

# How to run the truck #

## 0. You will need: ##
* The truck
* Charged batteries
* A powerbank to power the pi while driving.
* Xbox or PS3 controller
* Laptop with ROS installed and the right repos (see installation above)
* Connect to vnet on the truck and the laptop
* To get access to internet while connected to the vnet wifi you need to give Thomas the MAC-address for your computer. 
This is not needed to run the truck but will make your life easier

## 1. The truck ##
   * Boot the Pi by plugging in the powerbank
   * Turn on the motor
   * Wait for it to boot, run `roslaunch truck_hw_api hw_api.launch` 
        - This can be done by connecting to the truck via ssh `ssh pi@TRUCK_IP` and then running the commands above
   
## 2. The camera-system: ##

* Open a new terminal window and run:

 
    `ssh 192.168.5.31 -X -l bachelor`


    (Password is six characters, similar to the vnet password)
   

    `cd repository/visionlocalization_old/build`


    `./startCameras.sh LAPTOP_IP`

   
## 3. The laptop ##
* Plugin the xbox controller.
    * If you wanna use the ps3 controller, try this: [http://askubuntu.com/questions/409761/how-do-i-use-a-ps3-sixaxis-controller-with-ubuntu-to-control-games](http://askubuntu.com/questions/409761/how-do-i-use-a-ps3-sixaxis-controller-with-ubuntu-to-control-games)


* Run 


    `export ROS_HOSTNAME=YOUR_IP`
  

    `export ROS_MASTER_URI=http://TRUCK_IP:11311` 


    with correct IP addresses. Check ip with `ifconfig`. You need to export this in every new terminal window.

    To avoid the hassle of always having to enter these, you can add them to your .bashrc

    `#!bash`

    `echo "export ROS_HOSTNAME=YOUR_IP" >> ~/.bashrc`

    `echo "export ROS_MASTER_URI=http://TRUCK_IP:11311 >> ~/.bashrc `

    Then these variables will be exported each time you start a new terminal

* IMPORTANT: Before running the next command, make sure the back wheels are elevated so that it doesn't drive off.
* `roslaunch truck_master master.launch sim:=0`

     (this launches all nodes except the hardware API and the simulator node)

     * If you are using the ps3 controller, you need to add `gamepad_type:="ps3"` to the launch command
     * To launch with fast settings for pathplanning (if using slow computer, or running on the pi), add `rpi:=1`
   
## 4. How to drive: ##

* Set goals and subgoals just like in the simulator

![Controls.png](https://bitbucket.org/repo/nqxL85/images/3204438201-Untitled.png)

- The Dead Man's Switch needs to be pressed for the truck to move. (Applies to both manual and automatic driving)
- The select button changes the behavior of the gas pedal (nothing else), if it is used for reversing or driving forwards.

## 5. How to kill a node without killing all nodes: ##
    
* Run 

    `rosnode kill *nodename* `

* Change what you want to change in the source-code and start it again:

    `rosrun package_name node.py`
 
* Or run the python file directly.

    `cd /path/to/scripts`

    `python node.py`

## 6. Troubleshooting:  

* What is the password?
    - Ask in Slack
* Nothing happens when pressing the controller?
    - This might be a variety of issues, 
    - Echo rostopics to see where the issue is.

        `rostopic echo joy`

        `rostopic echo man_drive`

        `rostopic echo dead_mans_switch`

        `rostopic echo master_drive`

    - Double check if the `ROS_HOSTNAME` and `ROS_MASTER_URI` are set 
        - You can do this by running `echo $ROS_****`, if the output is empty then do the export again and restart everything
    - Check what the controller you are trying to use is called.
        * Unplug controller 
        * `ls /dev/input`
        * Plug in the controller
        * `ls /dev/input`
        * The output of this should have changed and something like "js0" will should have appeared
        * If the controller is not recognized as "js0", but as something else, for example "js1", you need to start master.launch like this:

            `roslaunch truck_master master.launch gamepad_input:="/dev/input/js1"`

    - No data is being published on "error"
        - make sure there is a visible tag ;)
    - I get a broken pipe error
        - start the hardware API from the truck instead of through ssh
        - run everything in screens, they do not quit when you loose the ssh session. 
        Take a look at [this link](https://www.rackaid.com/blog/linux-screen-tutorial-and-how-to/) for more information