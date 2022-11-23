#!/usr/bin/env python

import random
import math
import time
import rospy
import smach
from std_msgs.msg import Bool
from armor_api.armor_client import ArmorClient

mapflag = 0    #0 map is not loaded, 1 map is loaded
batflag = 1    #0 bat is low, 1 bat is full
urgentflag = 1 #0 urgent visit , 1 not urgent
sleeptime =2
stayinroomtime = 0.5

def callbackbattery(data):
    global batflag
    if data.data == 1:
        batflag = 1
    elif data.data ==0:
        batflag = 0

def callbackmap(data):
    global mapflag
    if data.data == 1:
        mapflag = 1
    elif data.data ==0:
        mapflag = 0

def urgentupdate():
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
   for i in list:
    try:
        start = i.index('"') + len('"')
        end = i.index('"', start)
        return i[start:end]
    except ValueError:
        return ""

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


class waiting_for_map(smach.State):
    global batflag
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepwaiting','maploaded'])

    def execute(self, userdata):
        client = ArmorClient("example", "ontoRef")
        rospy.sleep(sleeptime)
        if mapflag == 0:
            return 'keepwaiting'
        else:
            client.call('LOAD','FILE','',['/root/ros_ws/src/fsm_robot/my_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
            print("MAP IS LOADED...")
            return 'maploaded'

class move_in_corridor(smach.State):
    global batflag
    global urgentflag
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepmoving','battlow','urgentvisit'])

    def execute(self, userdata):
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

class charing(smach.State):
    global batflag
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepcharging','battfull'])

    def execute(self, userdata):
        client = ArmorClient("example", "ontoRef")
        print('Executing state: charing')
        rospy.sleep(sleeptime)
        if batflag == 1:
            print("BATTERY IS CHARGED...")
            return 'battfull'
        else:
            moveto('E')
            return 'keepcharging'

class visitroom(smach.State):
    global batflag
    global urgentflag
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepvisiting','noturgentvisit', 'battlow'])

    def execute(self, userdata):
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
    rospy.init_node('Robot_FSM')
    # Create a SMACH state machine
    robot = smach.StateMachine(outcomes=['Interface'])
    # Open the container
    with robot:
        # Add states to the container
        smach.StateMachine.add('waiting_for_map', waiting_for_map(), 
                               transitions={'keepwaiting':'waiting_for_map','maploaded':'move_in_corridor'})
        smach.StateMachine.add('move_in_corridor', move_in_corridor(), 
                               transitions={'keepmoving':'move_in_corridor','battlow':'charing','urgentvisit':'visitroom'})
        smach.StateMachine.add('charing', charing(), 
                               transitions={'keepcharging':'charing','battfull':'move_in_corridor'})
        smach.StateMachine.add('visitroom', visitroom(), 
                               transitions={'keepvisiting':'visitroom','noturgentvisit':'move_in_corridor','battlow':'charing'})
    
    rospy.Subscriber("batterylevel", Bool, callbackbattery)
    rospy.Subscriber("mapsituation", Bool, callbackmap)
    # Execute SMACH plan
    outcome = robot.execute()


if __name__ == '__main__':
    main()
    