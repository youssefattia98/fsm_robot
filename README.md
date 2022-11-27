# Description Logic & Finite State Machine Robot  

# [Sphinx](https://youssefattia98.github.io/fsm_robot/)


# 1) Introduction
This is a ROS package that simulates the architecture of a robot building an ontology of the map environment and uses finite state machine to explore the environment in an organized manner.  
The package depends on [Armor package](https://github.com/EmaroLab/armor). Furthermore, for the finite state machine the [Smach package](http://wiki.ros.org/smach) is used to set it up. Moreover, a detalied docmentation of the scripts can be found [here built with Sphinx](https://youssefattia98.github.io/fsm_robot/).

Please note, this package was devloped on a [docker image](https://hub.docker.com/r/carms84/exproblab). with all the dependieces needed pre-installed on the image.  




# 2) Software Architecture 
## I) Robot Behavior:  
First the robot waits for the ontology (map) to be built in Room E. Then starting from the (move_in_corridor) state it checks if the battery is not low or there is no urgent room, it moves randomly in the two corridors and wait for some time. However, if a battery is low it goes to the state (charing), which keeps the robot in room E and stayes there untill the battery is charged. Also, if there is an urgent room while the battery is charged the robot visits it and stays there for some time (visitroom state).  
The following digram shows the map Onltogy that the robot builds:  
![immagine](https://github.com/youssefattia98/fsm_robot/blob/main/docs/Digrams%20%26%20videos/MAP.PNG)  


## II) Finite State Machine digram:  
The following finite state machine shows the behavior the robot follows when the architecture was initially desigend:  
![immagine](https://github.com/youssefattia98/fsm_robot/blob/main/docs/Digrams%20%26%20videos/fsm_digram.PNG)  

## III) Nodes digram:    
The following digram shows the software architecture of the package as the main node here is the *finitesates node* and it requries three inputs to decide what the robot should do. 
* Map situation and this is retrived from the topic *mapsituation* and the *Ontlogybuild* node publishes on this topic.
* Battery situation and this is retrived from the topic *batterylevel* and the batterynode* node publishes on this topic.
* Room urgency as this is a global varible in the *finitesates node* that is changed to true or faluse according to the reposnse of armor service.  

![immagine](https://github.com/youssefattia98/fsm_robot/blob/main/docs/Digrams%20%26%20videos/sofar.drawio.png)  




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
## I) Video with smach viewer:
The following video shows the package running in action, also in parallel the smach viewer that shows each state the robot is in. Please note that the ontology building have already been made and this is only the fnite state machine working with armor. If you compare this running digram with the one presented previously in the Software Architecture section you would see that the finitestates node performs exactly the designed architecture.
The smach viewer can be run with the following commad:

```bash
$ sudo apt-get install ros-noetic-smach-viewer
$ rosrun smach_viewer smach_viewer.py
```

https://user-images.githubusercontent.com/69837845/203873910-f42bc6ac-4ba5-43d1-a721-ae874e73adc1.mp4    

The music used in this video I have no copy right on it. However, I think such a masterpiece of music should be listened to least a glimpse of it.  


## II) Rqt digram:  

The following digram shows thr rqt grapgh of the package running and how the noedes communicate with each other. Moreover, the pakage behaviour is as same as the desigend software architecture design and mentioned in the above point *2)III)*. The following comands can be used to view the nodes rqt digram:


```bash
$ sudo apt-get install ros-noetic-rqt
$ rosrun rqt_graph rqt_graph
```

![immagine](https://github.com/youssefattia98/fsm_robot/blob/main/docs/Digrams%20%26%20videos/rqt_grapgh_nodes.jpg)  


# 5) Working Hypothesis & Environment
## I) System’s Features
* The package is able to showcase the capablities of smach packge combined with armor packge.

* The package runs the robot in autonomous mode with random sleeping times between visits and battery state.
## II) System’s Limitations
* The package is designed to build the map as prsented before, therfore the behaviour of the robot is according to the sepecifed map. Therfore, if another map onlogy is built the package will not work properly. This is due to the my limited understanding of thr armor package docmentation. 

* As mentioned before, the package launch file opertes the robot in autonomous mode there for the user has not input for the battery state or map situation state. However, all the states was tested and the video above in point *4)I)* showes so.

## III) Future Imporvments
* Upgrade *moveto* function:  
    This fucntion is respoinble for moving the robot from it's current postion to another upgrading *isIn*,*now* and *visitedAt* properties. it is not a generic function and uses the understanding of the used map to move the robot correctly. However, using the *connected To* property would make the fucntion more generic and applipble for other maps.  

    ```python
    def moveto(newloction):
    client = ArmorClient("example", "ontoRef")
    #Update robot isin property
    client.call('REASON','','',[''])
    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req.queried_objects)

    if oldlocation== 'R1' or oldlocation == 'R2':
        print("I am moving from: " + oldlocation, "to: " + newloction)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1','C1',oldlocation])
    elif oldlocation == 'R3' or oldlocation == 'R4':
        print("I am moving from: " + oldlocation, "to: " + newloction)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1','C2',oldlocation])
    client.call('REASON','','',[''])
    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req.queried_objects)
    if oldlocation == 'C1' and (newloction== 'R3' or newloction =='R4'):
        print("I am moving from: " + oldlocation, "to: " + newloction)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1','C2','C1'])
    elif oldlocation == 'C2' and (newloction== 'R1' or newloction =='R2'):
        print("I am moving from: " + oldlocation, "to: " + newloction)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1','C1','C2'])
    
    client.call('REASON','','',[''])
    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req.queried_objects)
    print("I am moving from: " + oldlocation, "to: " + newloction)
    client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',newloction,oldlocation])
    
    #Update robot now property
    client.call('REASON','','',[''])
    req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    oldtimerobot=findtime(req.queried_objects)
    newtime=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', newtime, oldtimerobot])

    #Update the location visited at property
    client.call('REASON','','',[''])
    
    if newloction!= 'C1' and  newloction!='C2' and  newloction!= 'E':
        req=client.call('QUERY','DATAPROP','IND',['visitedAt', newloction])
        oldtimelocation=findtime(req.queried_objects)
        client.call('REPLACE','DATAPROP','IND',['visitedAt', newloction, 'Long', newtime, oldtimelocation])
        client.call('REASON','','',[''])
    ```
* Creating a manual mode launch file:  
    Apllying such solution would give the user the capability to design his/her own map and pass it to the package, also gives the capability of testing diffrent situations to see how the robot behaves.
* creating a seprate node for urgency checking:  
    Creating such a node would be a better software architecture as the *finitestate* node will be only resposible for the robot behavior and the urgency will be passed by a message through a topic that the **finitestate* node subscribes to.

* Upgrade the *Ontologybuild* behaviour:  
    As this nodes visits each room to update its *visitedAt* property it makes the algorithm not generic and only works for the specifed map. Instead the following command can be used to update the *visitedAt* property without having the inconsistency problem.  
    ```python
    client.manipulation.add_dataprop_to_ind("visitedAt", "R1", "Long", str(int(time.time())))
    ```



# 6) Authors and contacts
* Name: Youssef Attia
* Emmail: youssef-attia@live.com
* Linkedin: https://www.linkedin.com/in/youssefattia98/
