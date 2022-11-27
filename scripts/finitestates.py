#!/usr/bin/env python
"""
.. module:: finitestates
    :platform: Unix
    :synopsis: Python module for running the finite state machine of the robot

.. moduleauthor:: Youssef Attia youssef-attia@live.com
This node handles the main robot behavior, first it waits for the ontology (map) to be built.  

Then starting from the (move_in_corridor) state it checks if the battery is not low or there is no urgent room, it moves randomly in the two corridors and wait for some time.  

However, if a battery is low it goes to the state (charging), which keeps moves the robot in room E and stayes there untill the battery is charged
Also, if there is an urgent room while the battery is charged the robot visits it and stays there for some time (visitroom state).
"""


import roslib
import random
import math
import time
import rospy
import rospkg
import smach
import smach_ros
from std_msgs.msg import Bool
from armor_api.armor_client import ArmorClient

"""
Inherit the package pass and setes the .owl file pass 
"""
r = rospkg.RosPack()
path = r.get_path('fsm_robot')
newontology = path + "/Ontologies/my_map.owl"  


"""
Global Variables used to understand the map, battery and urgent room situation. Also, set the sleeping time in each room.  

"""
mapflag = 0
batflag = 1
urgentflag = 1
sleeptime =2
stayinroomtime = 0.5

def callbackbattery(data):
    """
    Function is the callback for the topic *batterylevel* and sets the global varible *batflag*.  

    Args:
        Battery state(class Bool): The data recived from the message.  

    Returns:
        void
    """
    global batflag
    if data.data == 1:
        batflag = 1
    elif data.data ==0:
        batflag = 0

def callbackmap(data):
    """
    Function is the callback for the topic *mapsituation* and sets the global varible *mapflag*.  

    Args:
        Map state(class Bool): The data recived from the message.  
        
    Returns:
        void
    """
    global mapflag
    if data.data == 1:
        mapflag = 1
    elif data.data ==0:
        mapflag = 0

def urgentupdate():
    """
    Function for checking if there is an urgent room to set the global *urgentflag*, also returns the nearby urgent room.  

    Args:
        void  

    Returns:
        Urgent room(string): The nearby urgent room according to the robot position in the corridors.
    """
    global urgentflag
    tobetrturned = '0'
    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    req=client.call('QUERY','IND','CLASS',['URGENT'])
    req2=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req2.queried_objects)
    for i in req.queried_objects:
        if oldlocation=='E':
            if random.randint(1, 2)==1:
                moveto('C1')
            else:
                moveto('C2')
            client.call('REASON','','',[''])
            req2=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            oldlocation=findindividual(req2.queried_objects)
        if oldlocation == 'C1':
            if "R1" in i:
                urgentflag = 0
                tobetrturned = 'R1'
                break
            elif "R2" in i:
                urgentflag = 0
                tobetrturned = 'R2'
                break
        elif oldlocation == 'C2':
            if "R3" in i:
                urgentflag = 0
                tobetrturned = 'R3'
                break
            elif "R4" in i:
                urgentflag = 0
                tobetrturned = 'R4'
                break
    if  tobetrturned == '0':
        urgentflag = 1
    else:
        return tobetrturned

def findindividual(list):
    """
    Function for finding the individual in a list from the return of a qureied proprity from armor.  

    Args:
        Individual(list): The individual in the armor resonse format ex. *['<http://bnc/exp-rob-lab/2022-23#R1>']*  

    Returns:
        Individual(string): The individual extarcted and changed to a string *ex. "R1"*
    """
    for i in list:
        if "R1" in i:
            return 'R1'
        elif "R2" in i:
            return 'R2'
        elif "R3" in i:
            return 'R3'
        elif "R4" in i:
            return 'R4'
        elif "C1" in i:
            return 'C1'
        elif "C2" in i:
            return 'C2'
        elif "E" in i:
            return 'E'

def findtime(list):
    """
    Function for finding the time with Unix format from the return of a qureied proprity from armor.  

    Args:
        Time(list): The time in the armor resonse format *ex. ['"1669241751"^^xsd:long']*  

    Returns:
        Time(string): The time extarcted and changed to a string *ex. "1665579740"*
    """
    for i in list:
        try:
            start = i.index('"') + len('"')
            end = i.index('"', start)
            return i[start:end]
        except ValueError:
            return ""

def moveto(newloction):
    """
    Function for changing to robot *isIn* property according to where is robot is. First it quires the robot location if the robot was in
    R1 or R2 it moves it to C1 else if the robot was in R3 or R4 it moves it to C2. And from there the robot can reach the nearby urgent rooms,
    the E charging room or go to the other corridor. Furthermore, updates the robot *now* property and also the location *visitedAt* property.  

    NOTE: This function is not generic, by other means can not be used for other ontologies as it depends mainly on the architecture of the currently
    used map. However, using the *connectedTo* property a more general function can be implemented.  

    Args:
        New loction(string)  

    Returns:
        void
    """
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


class waiting_for_map(smach.State):
    """
    Class for state *waiting_for_map* in which the robot waits for the *mapflag* to be True and then load the ontlogy.  

    Returns:
        *keepwaiting* if map is not loaded and *maploaded* if the map is loaded.  
        
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepwaiting','maploaded'])

    def execute(self, userdata):
        global mapflag
        client = ArmorClient("example", "ontoRef")
        rospy.sleep(sleeptime)
        if mapflag == 0:
            return 'keepwaiting'
        else:
            client.call('LOAD','FILE','',[newontology, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
            print("MAP IS LOADED...")
            return 'maploaded'

class move_in_corridor(smach.State):
    """
    Class for state *move_in_corridor* in which the robot checks if the battery is not low or there is no urgent room, it moves randomly in the two corridors and wait for some time.  

    Returns:
        *keepmoving* if the battery is not low and there is no urgent room, *battlow* if the batery is low and *urgentvisit* if there is an urgent room
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepmoving','battlow','urgentvisit'])

    def execute(self, userdata):
        global batflag
        global urgentflag
        client = ArmorClient("example", "ontoRef")
        urgentupdate()
        rospy.sleep(sleeptime)
        if batflag == 0:
            print("BATTERY IS LOW...")
            return 'battlow'
        if urgentflag == 0 and batflag ==1:
            print("THERE IS AN URGENT ROOM...")
            return 'urgentvisit'
        else:
            if random.randint(1, 2)==1:
                moveto('C1')
                rospy.sleep(stayinroomtime)
            else:
                moveto('C2')
                rospy.sleep(stayinroomtime)
            return 'keepmoving'

class charging(smach.State):
    """
    Class for state *charging* in which the moves to room E and stayes there untill the battery is charged.  

    Returns:
        *keepcharging* if the battery is still low, *battfull* if the batery is fully charged by other means the *batflag* is True.
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepcharging','battfull'])

    def execute(self, userdata):
        global batflag
        client = ArmorClient("example", "ontoRef")
        rospy.sleep(sleeptime)
        if batflag == 1:
            print("BATTERY IS CHARGED...")
            return 'battfull'
        else:
            moveto('E')
            return 'keepcharging'

class visitroom(smach.State):
    """
    Class for state *visitroom* in which the robot checks the nearby urgent rooms and visit them staying some time. Only if the battery is charged.  

    Returns:
        *keepvisiting* if the battery is still charged and there are nearby urgent room,  
        *noturgentvisit* if there is not urgent room by other means: visited all the nearby urgent rooms,  
        *battlow* if the battery is low.  

    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepvisiting','noturgentvisit', 'battlow'])

    def execute(self, userdata):
        global batflag
        global urgentflag
        client = ArmorClient("example", "ontoRef")
        urgentupdate()
        rospy.sleep(sleeptime)
        if urgentflag == 1:
            return 'noturgentvisit'
        elif batflag ==0:
            return 'battlow'
        else:
            The_urgnet_room=urgentupdate()
            moveto(The_urgnet_room)
            rospy.sleep(stayinroomtime)
            return 'keepvisiting'

def main():
    """
    This function initializes the ROS node, creates the finite state machine using `SMACH package: <http://wiki.ros.org/smach>`_. Also it subscries to both
    *batterylevel* and *mapsituation* topics.
    """
    rospy.init_node('Robot_FSM')
    # Create a SMACH state machine
    robot = smach.StateMachine(outcomes=['Interface'])
    # Open the container
    with robot:
        # Add states to the container
        smach.StateMachine.add('waiting_for_map', waiting_for_map(), 
                               transitions={'keepwaiting':'waiting_for_map','maploaded':'move_in_corridor'})
        smach.StateMachine.add('move_in_corridor', move_in_corridor(), 
                               transitions={'keepmoving':'move_in_corridor','battlow':'charging','urgentvisit':'visitroom'})
        smach.StateMachine.add('charging', charging(), 
                               transitions={'keepcharging':'charging','battfull':'move_in_corridor'})
        smach.StateMachine.add('visitroom', visitroom(), 
                               transitions={'keepvisiting':'visitroom','noturgentvisit':'move_in_corridor','battlow':'charging'})
    
    rospy.Subscriber("batterylevel", Bool, callbackbattery)
    rospy.Subscriber("mapsituation", Bool, callbackmap)
    # Execute SMACH plan
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', robot, '/SM_ROOT')
    sis.start()
    outcome = robot.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
    