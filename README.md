# Description Logic & Finite State Machine Robot

# 1) Introduction
This is a ROS package that simulates the architecture of a robot building an ontology of the map environment and uses finite state machine to explore the environment in an organized manner.  
The package depends on [Armor package](https://github.com/EmaroLab/armor). Furthermore, for the finite state machine the [Smach package](http://wiki.ros.org/smach) is used to set it up. Moreover, a detalied docmentation of the scripts can be found [here built with Sphinx](https://youssefattia98.github.io/fsm_robot/).

Please note, this package was devloped on a [docker image](https://hub.docker.com/r/carms84/exproblab). with all the dependieces need pre-installed on the image.  




# 2) Software Architecture 
## I) Robot Behavior:  
First it waits for the ontology (map) to be built in Room E. Then starting from the (move_in_corridor) state it checks if the battery is not low or there is no urgent room, it moves randomly in the two corridors and wait for some time. However, if a battery is low it goes to the state (charing), which keeps the robot in room E and stayes there untill the battery is charged. Also, if there is an urgent room while the battery is charged the robot visits it and stays there for some time (visitroom state).  
The following digram shows the map Onltogy that the robot builds:  
![immagine](https://github.com/youssefattia98/fsm_robot/blob/main/docs/Digrams%20%26%20videos/MAP.PNG)  


## II) Finite State Machine digram:  
The following finite state machine shows the behavior the robot follows when the architecture was initially desigend:  
![immagine](https://github.com/youssefattia98/fsm_robot/blob/main/docs/Digrams%20%26%20videos/fsm_digram.PNG)  

## III) nodes digram




# 3) Installation
For setting up the enviormnt for this pakage to run correctly [Armor package](https://github.com/EmaroLab/armor) and [Smach package](http://wiki.ros.org/smach). so please check their docmenation for the installation.  

On the other hand, *xtrem* is needed as the launch file requires it and can be installed from the following command:  
```bash
$ sudo apt-get -y install xterm
``` 
After the *xtrem* is installed, clone this repo in your ROS workspace and build it using catkin_make as following:
```bash
$ cd <your ROS ws/src>
$ git clone "https://github.com/youssefattia98/fsm_robot.git"
$ cd ..
$ catkin_make
```
As the package is built successfully, now the ROS launch file can be executed. Please note, for the current the package is setup to run autonomously to test the software architecture. However, further developments can be done on this package to require the user interface.
```bash
$ roslaunch fsm_robot launcheverything.launch
```



# 4) Package In Action  
The following video shows the package running in action, also in parallel the smach viewer that shows each state the robot is in. Please note that the ontology building have already been made and this is only the fnite state machine working with armor.  
The smach viewer can be run with the following commad:

```bash
$ rosrun smach_viewer smach_viewer.py
```

https://user-images.githubusercontent.com/69837845/203873910-f42bc6ac-4ba5-43d1-a721-ae874e73adc1.mp4    

The music used in this video I have no copy right on it. However, I think such a masterpiece of music should be listened to least a glimpse of it.


# 5) Working Hypothesis & Environment
## I) System’s Features
* is able to showcase the capablities of smach combined with armor
## II) System’s Limitations
* can not be applied on other onltogies
* no manual control
## III) Future Imporvments
* change move to function
* better launch file
* check urgnciy in a seprate node
* devlope a more general version



# 6) Authors and contacts
* Name: Youssef Attia
* Emmail: youssef-attia@live.com
* Linkedin: https://www.linkedin.com/in/youssefattia98/
