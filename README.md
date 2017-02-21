## What is this repository for? ##
For regulating commands to the hardware API, choosing between manual and automatic

# How to run the truck #

## 0. You will need: ##
* The truck
* A box to place it on, elevating the back wheels, so it doesn't drive off when starting
* Hdmi cable
* Charged batteries
* A powerbank to power the pi while driving.
* Xbox or PS3 controller
* Laptop with ROS installed
* All the latest versions of the repositories needed
    * Download this [script](https://drive.google.com/open?id=0BxRJZY1j9wVVMDBwVDBBRG12cTA), which creates a new catkin workspace and clones down all the repositories you need

    * Put the file where you want to create the new workspace and source it
     
          `cd path/to/trucker`      

          `source ./trucker`

* Ackermann_msgs and Joy packages installed: 

    `sudo apt-get update && sudo apt-get upgrade`

    `sudo apt-get install ros-kinetic-ackermann-msgs`

    `sudo apt-get install ros-kinetic-joy`


* Connect to vnet on the truck and the laptop
* To get access to internet while connected to the vnet wifi you need to give Thomas the MAC-address for your computer. 
This is not needed to run the truck but will make your life easier

## 1. The truck ##
   * Turn off battery
   * Boot the Raspberry PI with HDMI plugged in, by plugging in the powerbank. (Don't forget to turn on projector)
   * Shortly after, turn on the batteries
   * Wait for it to boot, open two terminal windows, run `roscore` and in the other `rosrun truck_hw_api cmd_node.py` 
        - This can be done by connecting to the truck via ssh `ssh pi@TRUCK_IP` and then running the commands above
   
## 2. The camera-system: ##

* Open a new terminal window and run:

 
    `ssh 192.168.5.31 -X -l bachelor`


    (Password is six characters, similar to the vnet password)
   

    `cd repository/visionlocalization_old/build`


    `./script.sh`

   
## 3. The laptop ##
* Plugin the xbox controller.
    * If you wanna use the ps3 controller, try this: [http://askubuntu.com/questions/409761/how-do-i-use-a-ps3-sixaxis-controller-with-ubuntu-to-control-games](http://askubuntu.com/questions/409761/how-do-i-use-a-ps3-sixaxis-controller-with-ubuntu-to-control-games)


* Run 


    `export ROS_HOSTNAME=YOUR_IP`
  

    `export ROS_MASTER_URI=http://TRUCK_IP:11311` 


    with correct IP addresses. Check ip with `ifconfig`. You need to export this in every new terminal window.

    To avoid the hassle of always having to enter these, you can add them to your .bashrc
    ```
    #!bash
    echo "export ROS_HOSTNAME=YOUR_IP" >> ~/.bashrc
    echo "export ROS_MASTER_URI=http://TRUCK_IP:11311 >> ~/.bashrc 
    ```
    Then these variables will be exported each time you start a new terminal

* IMPORTANT: Before running the next command, make sure the back wheels are elevated so that it doesn't drive off.
* `roslaunch truck_master master.launch`

     (this launches all nodes except the hardware API)

     * If you are using the ps3 controller, you need to add `gamepad_type:="ps3"` to the launch command
   
## 4. How to drive: ##
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
*  If the PI turns off randomly while driving, it means there's an issue with the power. Either the batteries have ran out, or the batteries aren't strong enough to power both the motor and the pi.
    * Charge batteries
    * Use a powerbank to power the PI.
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
        - start the roscore and hardware API from the truck instead of through ssh
        - run everything in screens, they do not quit when you loose the ssh session. 
        Take a look at [this link](https://www.rackaid.com/blog/linux-screen-tutorial-and-how-to/) for more information