'''
.. module:: go_to_point
   :platform: Unix
   :synopsis: Node for implementing the simulation of the robot movement.
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

Node for implementing the simulation of the robot movement.

This node simulation the robot of the movement, it calculates the euclidean distance between the points and wait for the time to reach the distance (scaling of 1/10 is applied to reduce the wait time).
            
Service: 
     /move_point: Generates the movement.                   
'''

import rospy
import math
import time
from geometry_msgs.msg import Point
from assignment1.srv import Move, MoveResponse



def move(dist):
    '''
    This is the function simulate the motion of the robot. When the funciton is called it wiat for the 0.1 of the time needed to travel the euclidean distance calculated between two points. 
    '''

    print("\nDistance to the next position: " + str(dist))
    time.sleep(0.1*dist) 
    print("\nTarget reached!")
    
    return True


def get_target(pos):
    '''
    This is the function receives callback from the state machine to execute the motion of the robot. Once the state machine send the request, it calculates the euclidean distance between the point and call the move funciton to wait for the time taken to reach the distance.
    '''

    actual_pos=Point()
    target=Point()

    actual_pos.x=pos.x_start
    actual_pos.y=pos.y_start

    target.x=pos.x_end
    target.y=pos.y_end

    # Compute Euclidean distance
    dist = math.sqrt(pow(target.x-actual_pos.x, 2)+pow(target.y-actual_pos.y, 2))

    # Simulate the motion by calling the move() function
    res=move(dist)
    msg=MoveResponse()
    msg.reached=res

    return msg


    

def main():
    '''
    This is the main fucniton of the move to point module intializes the module. And creates the /move_point topic
    '''

    rospy.init_node('go_to_point') 
    rospy.Service('/move_point', Move, get_target)

    rospy.spin()



if __name__ == '__main__':
    main()
