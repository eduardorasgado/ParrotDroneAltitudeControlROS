#! /usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Empty

#messages for action
from ardrone_altitude.msg import ArdroneAltitudeAction
from ardrone_altitude.msg import ArdroneAltitudeFeedback
from ardrone_altitude.msg import ArdroneAltitudeResult

class TakeoffOrLand:
    def __init__(self, takeoff=None, land=None):
        self.takeoff = takeoff
        self.land = land
        self.takeoff_msg = Empty()
        self.land_msg = Empty()
        
    def up(self):
        #METHOD to get up the drone engine
        self.takeoff = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
        
    def down(self):
        #method to land uthe drone engine
        self.land = rospy.Publisher("/drone/land", Empty, queue_size=1)
        
    def publish(self, choice):
        if choice:
            self.takeoff.publish(self.takeoff_msg)
            return True
        self.land.publish(self.land_msg)


class DroneAltitude:
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=4)
        self.move = Twist()
        self.rate1 = rospy.Rate(1)
    
    def up(self):
        #publish to msg for up the drone in z 
        self.move.linear.z = 0.2
        mr = self.publish_cmd_vel(self.move)
        if mr:
            rospy.loginfo("action up published")
        
    def down(self):
        #publish to msg for down the drone in z
        self.move.linear.z = -0.2
        mr = self.publish_cmd_vel(self.move)
        if mr:
            rospy.loginfo("action down published")
        
    def stop(self):
        #publish to msg for stop the drone in z
        self.move.linear.z = 0
        mr = self.publish_cmd_vel(self.move)
        if mr:
            rospy.loginfo("action stop published")
        
    def publish_cmd_vel(self, my_vel):
        #check if there are connections to publish
        connections = self.pub.get_num_connections()
        if connections > 0:
            try:
                #publishing the actions
                self.pub.publish(my_vel)
                rospy.loginfo("action published")
                return True
            except:
                rospy.logerr("An error ocurred moving the drone in server")
                return False
        else:
            #if no connections
            self.rate1.sleep()
            rospy.logerr("An error ocurred moving the drone in server: Connection to cmd_vel node could take place")
            return False

    
class DroneFlightServer:
    _feedback = ArdroneAltitudeFeedback()
    _result = ArdroneAltitudeResult()
    
    def __init__(self, pub = None, serverName="parrot_altitude_actionserver"):
        self.pub = pub
        self.serverName = serverName
        self._as = actionlib.SimpleActionServer(serverName, ArdroneAltitudeAction, self.goal_callback, False)
        self._as.start()
        
    def goal_callback(self, goal):
        #drone setup for start the flight and land the drone
        self.drone_takeoff_land = TakeoffOrLand()
        self.drone_takeoff_land.up()
        self.drone_takeoff_land.down()
        
        #up the drone
        self.takingoff_land_the_drone(self.drone_takeoff_land, 1)
        
        r = rospy.Rate(1)
        success = True
        
        self._feedback.feedback.data = ""
        
        rospy.loginfo("The movements are going to start, preceiviing orders from the client...")
        
        #creating the mover up/down 1 meter instance
        self.myDrone = DroneAltitude()
        timing = 0
        while timing <5:
            #check if not cancelled
            if self._as.is_preempt_requested():
                rospy.loginfo("The goal has been cancelled/preempted")
                #set a cancelled goal
                self._as.set_preempted()
                success = False
                break
                
            #call the object to up or down for one meter: 0.2 m/s then 5 seconds = 1 meter nearly
            if goal.goal.data == "UP":
                self.myDrone.up()
            elif goal.goal.data == "DOWN":
                self.myDrone.down()
            
            #lets send the feedback message
            self._feedback.feedback.data = goal.goal.data
            self._as.publish_feedback(self._feedback)
            
            #set one second to movement
            time.sleep(1)
            
            self.myDrone.stop()
            
            #frequency
            r.sleep()
            timing += 1
        
        #land the drone
        #self.takingoff_land_the_drone(self.drone_takeoff_land, 0)
        
        #handling the empty result
        if success:
            self._result = Empty()
            msg = "Drone landed, mission accomplished"
            rospy.loginfo(msg)
            #send the empty result 
            self._as.set_succeeded(self._result)
            
        
    def takingoff_land_the_drone(self, drone, up_or_down):
        i = 0
        #3 seconds to this action
        while not i == 5:
            drone.publish(up_or_down)
            rospy.loginfo("Taking Off...")
            time.sleep(1)
            i += 1
            
"""
For sending a goal manually:
    rostopic pub /parrot_altitude_actionserver/goal ardrone_altitude/ArdroneAltitudeActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  goal:
    data: 'UP'"
    
in goal data: UP or DOWN

for watching the feedback:
    rostopic echo /parrot_altitude_actionserver/feedback
    
After the caller has finished = 
    rostopic pub /drone/land std_msgs/Empty "{}"
"""


if __name__=="__main__":
    rospy.init_node("parrot_altitude_server_node")
    DroneFlightServer()
    rospy.spin()