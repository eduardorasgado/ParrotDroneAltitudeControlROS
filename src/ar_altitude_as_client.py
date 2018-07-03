#! /usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Empty

from ardrone_altitude.msg import ArdroneAltitudeAction
from ardrone_altitude.msg import ArdroneAltitudeFeedback
from ardrone_altitude.msg import ArdroneAltitudeResult
from ardrone_altitude.msg import ArdroneAltitudeGoal

class droneAltitudeClient:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4
    
    def __init__(self, up_or_down, action_server_name='/parrot_altitude_actionserver', action=ArdroneAltitudeAction):
        self.up_or_down = up_or_down
        self.action_server_name = action_server_name
        self.action = action
        
        self.nodo = rospy.init_node('parrot_altitude_client_node')
        self.client = actionlib.SimpleActionClient(self.action_server_name, self.action)
        
        self.waitingforServer(self.client)
        
        self.goalHandling(self.client, self.up_or_down)
        
        self.state_result = self.client.get_state()
        
        self.rate = rospy.Rate(1) #1Hz
        
        self.state_result = self.altitude_communication(self.state_result, self.client)
        
    def waitingforServer(self, client):
        rospy.loginfo("Waiting for action server..."+self.action_server_name)
        client.wait_for_server()
        rospy.loginfo("Action server found: "+self.action_server_name)
    
    def goalHandling(self, client, up_or_down):
        goal = ArdroneAltitudeGoal()
        goal.goal.data = up_or_down
        client.send_goal(goal, feedback_cb=self.feedback_callback)
        
    def feedback_callback(self, feedback):
        msg = '[Feedback] Action is: '+str(feedback.feedback.data)
        rospy.loginfo(msg)
        
    def altitude_communication(self, state_result, client):
        while state_result < self.DONE:
            rospy.loginfo("Receiving feedback")
            self.rate.sleep()
            state_result = client.get_state()
            msg = "state_result: "+str(state_result)
            rospy.loginfo(msg)
        return state_result
        
        
if __name__=="__main__":
    altitude = str(input("Insert the direction of the drone(UP/DOWN): "))
    if altitude == "UP" or altitude == "DOWN":
        droneAltitudeClient(altitude)
    else:
        print("Just UP or DOWN please")
        
        