#!/usr/bin/env python
"""
.. module:: Ontologybuild
    :platform: Unix
    :synopsis: Python module for building the map on the ontology

.. moduleauthor:: Youssef Attia youssef-attia@live.com
This node imports the main ontology topological_map.owl file which is provided form this `repo: <https://github.com/buoncubi/topological_map>`_.
Adds the locations and doors and disjoints them, later it makes the robot take a cruise in each room adding the *visitedAt* property for each of them
and also updating the robot *now property. This makes it easier for the node *finitestates* to replace these properties.  

Furthermore, the newly built ontology is saved on a separate file to be used from the *finitestates* node and a message is sent to the topic *mapsituation*
indicating that the map is built.
"""

import random
import time
import math
import rospy
import rospkg
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool

"""
Inherit the package pass and setes the .owl file pass 
"""
r = rospkg.RosPack()
path = r.get_path('fsm_robot')
oldontology = path + "/Ontologies/topological_map.owl"
newontology = path + "/Ontologies/my_map.owl"

"""
Global Variables used to set the random sleeping time between each visit, the *maxwait* is set as as the urgent room threshold is 7 and there is 4 wait periods so 7/4 = 1.75 sec
"""
minwait = 0.0
maxwait = 1.75

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

def build_Ontology():
   """
   Function for loading the Ontology, building it, visiting rooms, updating timestamps and saving the new Ontology  

   Args:
      void  
   Returns:
      void
   """

   print("Hello I am building the map...")
   client = ArmorClient("example", "ontoRef")
   pub = rospy.Publisher('mapsituation', Bool, queue_size=10)
   rospy.init_node('mapsituation_node', anonymous=True)
   pub.publish(0)

   client.call('LOAD','FILE','',[oldontology, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D6'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D7'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R1', 'D1'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R2', 'D2'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R3', 'D3'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R4', 'D4'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D1'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D2'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D5'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D6'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D3'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D4'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D5'])
   client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D7'])
   client.call('DISJOINT','IND','',['R1','R2','R3','R4','E','C1','C2','D1','D2','D3','D4','D5','D6','D7'])
   client.call('REASON','','',[''])

   print("I have built the Map, will take a tour in the rooms...")
   #Visit R1
   #Update robot isin property
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'R1','E'])
   #Update robot now property
   client.call('REASON','','',[''])
   req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
   oldtimerobot=findtime(req.queried_objects)
   newtime=str(math.floor(time.time()))
   client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', newtime, oldtimerobot])
   #Update the location visited at property
   client.call('REASON','','',[''])
   client.call('ADD','DATAPROP','IND',['visitedAt','R1', 'Long', newtime])
   client.call('REASON','','',[''])

   rospy.sleep(random.uniform(minwait, maxwait))
   #Visit R2
   #Update robot isin property
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'C1','R1'])
   client.call('REASON','','',[''])
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'R2','C1'])
   #Update robot now property
   client.call('REASON','','',[''])
   req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
   oldtimerobot=findtime(req.queried_objects)
   newtime=str(math.floor(time.time()))
   client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', newtime, oldtimerobot])
   #Update the location visited at property
   client.call('REASON','','',[''])
   client.call('ADD','DATAPROP','IND',['visitedAt','R2', 'Long', newtime])
   client.call('REASON','','',[''])

   rospy.sleep(random.uniform(minwait, maxwait))
   #Visit R3
   #Update robot isin property
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'C1','R2'])
   client.call('REASON','','',[''])
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'C2','C1'])
   client.call('REASON','','',[''])
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'R3','C2'])
   #Update robot now property
   client.call('REASON','','',[''])
   req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
   oldtimerobot=findtime(req.queried_objects)
   newtime=str(math.floor(time.time()))
   client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', newtime, oldtimerobot])
   #Update the location visited at property
   client.call('REASON','','',[''])
   client.call('ADD','DATAPROP','IND',['visitedAt','R3', 'Long', newtime])
   client.call('REASON','','',[''])

   rospy.sleep(random.uniform(minwait, maxwait))
   #Visit R4
   #Update robot isin property
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'C2','R3'])
   client.call('REASON','','',[''])
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'R4','C2'])
   #Update robot now property
   client.call('REASON','','',[''])
   req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
   oldtimerobot=findtime(req.queried_objects)
   newtime=str(math.floor(time.time()))
   client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', newtime, oldtimerobot])
   #Update the location visited at property
   client.call('REASON','','',[''])
   client.call('ADD','DATAPROP','IND',['visitedAt','R4', 'Long', newtime])
   client.call('REASON','','',[''])

   rospy.sleep(random.uniform(minwait, maxwait))
   #Visit E
   #Update robot isin property
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'C2','R4'])
   client.call('REASON','','',[''])
   client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', 'E','C2'])
   #Update robot now property
   client.call('REASON','','',[''])
   req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
   oldtimerobot=findtime(req.queried_objects)
   newtime=str(math.floor(time.time()))
   client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', newtime, oldtimerobot])

   print("Everything is fine, map built, saved and publishing to the map topic...")
   client.call('SAVE','','',[newontology])
   pub.publish(1)
   
if __name__ == '__main__':
   try:
      build_Ontology()
   except rospy.ROSInterruptException:
      pass 