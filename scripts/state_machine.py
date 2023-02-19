#! /usr/bin/env python

'''
.. module:: state_machine (CLUEDO FSM)
   :platform: Unix
   :synopsis: Node for implementing the cludo game simulation 
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

Node for implementing the cludo game simulation 

This node stimulate the three sates of the robot. The three states are move state, enter the room state and the solution check state. 
  
 1)MOVE: In the first state the robot starts from the oracle room and start to explore rooms in order to acquire hints. The Move custom service message is send to the go_to_point node in order to simulate the movement of the robot. And if is followed by any one of the following state.
 2)ENTER_ROOM: The robot enters the room and ask the hin to oracle node, oracle node provides the hint with the ID. Then it add the hints to the ontology by MYArmor class and checks for the consistency. If it is consistent, it goes to the oracle room to check the solution.
 3)SOLUTION: This is the state which the robot try to check the solution with the oracle note and based on the solution i.e whether it is correct or not. It is followed by the end of game solution or goes back to move node.
            
Client: 
     /move_point: custom service to send the position to reach to the go_to_point node
     /hint_request: custom service to ask a hint message to oracle node
     /solution: custom service to send the possible solution to the oracle node                      
'''


from posixpath import dirname, realpath
import rospy
from rospy.impl.tcpros_service import ServiceProxy, wait_for_service
import smach
import smach_ros
import random
from datetime import datetime
import time
from geometry_msgs.msg import Point
from assignment1.srv import Move, MoveRequest, AskHint, Solution, SolutionRequest
from classes.myArmor import MyArmor
from classes.place import Place
from armor_msgs.srv import *
from armor_msgs.msg import *

pub_move=None
''' 
Initialize the publisher to /move_point
'''
pub_ask_hint=None
''' 
Initialize the publisher to /hint_request
'''
pub_solution=None
''' 
Initialize the publisher to /solution
'''
places=[]
''' 
Array with the places of the scene
'''
actual_pos=None
''' 
Actual position of the robot in the environment
'''
oracle=False
''' 
Boolean to know if the robot must go to the Oracle_Room
'''
response_complete=None
''' 
Response of the query for a complete hypothesis
'''

people_ontology=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock']
''' 
Define all the people of the scene
'''
places_ontology=['Ballroom', 'Billiard_Room', 'Conservatory']
''' 
Define all the places of the scene
'''
weapons_ontology=['Candlestick', 'Dagger','LeadPipe']
''' 
Define all the weapons of the scene
'''
oracle_room=None
''' 
Initialize the Oracle_Room
'''



def init_scene():
    '''
    This is the function to initialize the scene with the three rooms and initiate the robot in start postion i.e orcale room. It adds all the information to the armor ontology and pefroms disjoint t0 seperate them respect to same class element. And at last it performs reason to update the ontology.    
    '''

    global places, actual_pos, oracle_room
    places.append(Place(places_ontology[0], 5, 5))
    places.append(Place(places_ontology[1], -5, -5))
    places.append(Place(places_ontology[2], 5, -5))

    oracle_room=Place('Oracle_Room', 0, 0)
    actual_pos=Place('Oracle_Room', 0, 0) 

    j=0
    while j!=len(people_ontology):
        res=MyArmor.add_item(people_ontology[j], 'PERSON')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(people_ontology[j], people_ontology[count-1])
                count = count -1
        j=j+1

    j=0
    while j!=len(places_ontology):
        res=MyArmor.add_item(places_ontology[j], 'PLACE')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(places_ontology[j], places_ontology[count-1])
                count = count -1
        j=j+1

    j=0
    while j!=len(weapons_ontology):
        res=MyArmor.add_item(weapons_ontology[j], 'WEAPON')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(weapons_ontology[j], weapons_ontology[count-1])
                count = count -1
        j=j+1

    res=MyArmor.reason()
    
        

class Explore(smach.State):
    '''
    This class is used to represent the movement of the robot in the environment from place to place. 
    '''

    def __init__(self):
        '''
        Initialise the state 
        '''
        smach.State.__init__(self, 
                             outcomes=['enter_room', 'solution'])
        
    def execute(self, userdata):
        '''
        This method, first it checks the condition to move to the oracle room or to the other place in the environment(to move randomly in places except oracel room). And it continuosly update the postion of the robot and does not allow to stay in the actual postion. Based on the solution obtained it switch to other states. It returns the value to move between the states. Arguments are userdata to store the vatiables between the states.
        '''
        global pub_move, actual_pos, places, oracle

        if oracle==False:
            random.seed(datetime.now())
            index=random.randint(0, len(places)-1)

            ## Can't be in the same place in which it is
            while places[index].name==actual_pos.name:
                index=random.randint(0, len(places)-1)
            destination=places[index]

        else:
            destination=oracle_room

        # Generate the Move custom message to perform the motion
        msg=MoveRequest()
        msg.x_start=actual_pos.x
        msg.y_start=actual_pos.y
        msg.x_end=destination.x
        msg.y_end=destination.y

        res=pub_move(msg)
        print("\nMoving from " + actual_pos.name + " to " + destination.name +"\n")

        # Wait for a response from the go_to_point node
        rospy.wait_for_service('move_point')

        if(res.reached==True):
            # When reached update the current position
            actual_pos.x=destination.x
            actual_pos.y=destination.y
            actual_pos.name=destination.name
        else:
            print("\nPosition " + destination.name + " not reached")

        # If it is going to the oracle room it sets oracle to False for the future iterations of the state_machine, then returns 'solution'
        if oracle == True:
            oracle=False
            return 'solution'
        
        else:
            return 'enter_room'


class Enter_Room(smach.State):
    '''
        This is a class used to represent the behavior of the robot when it enters in a room and asks for hints.
    '''

    def __init__(self):
        '''
        Initialise the state 
        '''

        smach.State.__init__(self, 
                             outcomes=['explore'])
        
    def execute(self, userdata):
        '''
        This method asks for the hint from the oracle node and send them to oracle to check for consistency whether the hint are complete and consistent hypothesis in the ontology. Then it moves to the oracel to check the solution if it is true or otherwise the hypothesis are removed. The return is string to switch to the move state. And arguments are same as the move state.
        '''

        global pub_ask_hint, oracle, response_complete

        res=pub_ask_hint()  
        # ID of the hint received
        ID=res.ID

        # Add hints to the ontology 
        count=0
        while(count!=len(res.what)):
            request=MyArmor.add_hypothesis('what', ID, res.what[count])

            if request.armor_response.success==False:
                print("\nError, cannot add " + res.what[count])
            count = count +1

        count=0
        while(count!=len(res.where)):
            request=MyArmor.add_hypothesis('where', ID, res.where[count])

            if request.armor_response.success==False:
                print("\nError, cannot add " + res.where[count])
            count = count +1

        count=0
        while(count!=len(res.who)):
            request=MyArmor.add_hypothesis('who', ID, res.who[count])

            if request.armor_response.success==False:
                print("\nError, cannot add " + res.who[count])
            count = count +1

        # Reason
        reason=MyArmor.reason()

        if reason.armor_response.success==False:
                print("\nError, cannot perform reasoning")


        # First asks for all consistent queries
        response_complete=MyArmor.ask_complete()
        if response_complete.armor_response.success==False:
            print("\nError in asking query")        

        if len(response_complete.armor_response.queried_objects)!=0:
            response_inconsistent=MyArmor.ask_inconsistent()
        
            # Look for possible inconsistent hypothesis and in case remove them
            if len(response_inconsistent.armor_response.queried_objects)!=0:
                str_inconsistent=response_inconsistent.armor_response.queried_objects[0]
                str_inconsistent=str_inconsistent[40:]
                id_inconsistent=str_inconsistent[:-1]
                print("ID_inconsistent "+str(id_inconsistent) +"\n")
                res=MyArmor.remove(id_inconsistent)
                
                if res.armor_response.success==False:
                    print("Error in removing\n")
            
            ## If the hypothesis are completed and not inconsistent, then let's go to the oracle
            else:
                oracle=True
        # If the response isn't complete it removes it from the ontology
        else:
            res=MyArmor.remove(ID)
            if res.armor_response.success==False:
                print("Error in removing\n")

        return 'explore'
            



class Try_Solution(smach.State):
    '''
        This is a class used to represent the behavior of the robot when it tries to generate a solution
    '''

    def __init__(self):
        '''
        Initialise the state 
        '''
        smach.State.__init__(self, 
                             outcomes=['explore', 'correct'])
        
    def execute(self, userdata):
        '''
        This execute method checks the solution with the oracel node by sending the the solution request. And if it is true the game simulation end or else the it removed them from the ontology. It saves the solution if it is true or else it switch to the move state. Returns the string to stop the simulation and string to switch to the move state accordingly.
        '''

        global  pub_solution

        #In order to make the view simulation more comfortable wait 1 second before the execution
        time.sleep(1) 
        
        # Get the id of the hypothesis
        id_consistent=response_complete.armor_response.queried_objects[0]
        id_consistent=id_consistent[40:]
        id_consistent=id_consistent[:-1]

        print("\n\nID consistent: " + id_consistent + "\n")

        # Get the weapon
        res_what=MyArmor.ask_item('what', id_consistent)
        what=res_what.armor_response.queried_objects[0]
        what=what[40:]
        what=what[:-1]

        # Get the place
        res_where=MyArmor.ask_item('where', id_consistent)
        where=res_where.armor_response.queried_objects[0]
        where=where[40:]
        where=where[:-1]

        # Get the person
        res_who=MyArmor.ask_item('who', id_consistent)
        who=res_who.armor_response.queried_objects[0]
        who=who[40:]
        who=who[:-1]

        # Formalize the possible solution
        print("\nThe killer is: " + who + " in the " + where + " with " + what + "\n")

        # Send the solution to oracle
        sol=SolutionRequest()
        sol.what=what
        sol.where=where
        sol.who=who
        res=pub_solution(sol)
        rospy.wait_for_service('solution')
        if res.correct==True:
            print("\nSolution is correct, the game is finished!\n")
            MyArmor.save()
            return 'correct'

        elif res.correct==False:
            print("\nIt's not the correct solution\n")
            res=MyArmor.remove(id_consistent)

            return 'explore'
    


def main():
    '''
    The main function of the node. It initialises the node at first. Followed by initialising the services to /move_point,/hint_request and /solution topics. Then it loads cluedo ontology soutce file with the help of MyArmor class and then initilise the scence. At last it defines the sates of the state machine with smach.
    '''

    global  pub_move, pub_ask_hint, pub_solution

    # Initialize the node
    rospy.init_node('state_machine')

    pub_move=ServiceProxy('/move_point', Move)
    pub_ask_hint=ServiceProxy('/hint_request', AskHint)
    pub_solution=ServiceProxy('/solution', Solution)

    # Gets the actual path of the state_machine.py file
    path = dirname(realpath(__file__))
    path = path[:-7] + "cluedo_ontology.owl"
    
    # Loads the ontology
    response=MyArmor.load(path)

    if response.armor_response.success==True:
        print("\nOntology loaded successfully")
    else:
        print("\nERROR: Ontology load error")

    # Initialize the scene
    init_scene()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['state_machine'])
    sm.userdata.sm_counter = 0


     # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Explore(), 
                               transitions={'enter_room':'ENTER_ROOM',
                                            'solution':'SOLUTION' })

        smach.StateMachine.add('ENTER_ROOM', Enter_Room(), 
                               transitions={'explore':'MOVE'})
        
        smach.StateMachine.add('SOLUTION', Try_Solution(),
                                transitions={'explore':'MOVE',
                                             'correct':'state_machine'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
