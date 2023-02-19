#! /usr/bin/env python

'''
.. module:: oracle
   :platform: Unix
   :synopsis: Node for implementing the cludo oracle room
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

Node for implementing the cludo room, the node which has the solution and generate the hints.

This node generates the solution of the cludeo game randomly. ANd generate hints randomly when the state machine makes the request. And when the state machine sends the solution it checks whether the solution is true and sends the appropriate return message.  
            
service: 
     /solution : generates the solution.
     /hint_request : provide hints to the state machine module.                      
'''


import rospy
import random
from datetime import datetime
from assignment1.srv import Solution, SolutionResponse, AskHint, AskHintResponse

people=['Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock']
''' 
Define all the people of the scene
'''
places=['Ballroom', 'Billiard_Room', 'Conservatory']
''' 
Define all the places of the scene
'''
weapons=['Candlestick', 'Dagger','LeadPipe']
''' 
Define all the weapons of the scene
'''
solution=[]
''' 
Define the array in which will be stored the solution of the game
'''
num_ID_hint=0
''' 
To count the number of hints provided by the oracle node
'''

def init_scene():
    '''
    This funciton creates the random solution and saves them in the solution variable whent the game initialized.
    '''

    global people, places, weapons, solution

    index=[]

    random.seed(datetime.now())
    index.append(random.randint(0, len(weapons)-1))
    index.append(random.randint(0, len(places)-1))
    index.append(random.randint(0, len(people)-1))

    solution.append(weapons[index[0]])
    solution.append(places[index[1]])
    solution.append(people[index[2]])

    print("Solution: "+str(solution))



def receive_solution(sol):
    '''
    This is the funciton which checks the predicted solution, when state machine sends the solution, it checks the solution and returns whether the solution is true or flase i.e corrct or not.
    '''

    print("\nSolution received: " + sol.what + ", " + sol.where + ", " + sol.who)
    
    res=SolutionResponse()
    if(solution[0]==sol.what and solution[1]==sol.where and solution[2]==sol.who):
        print("\nSolution is correct!!")
        res.correct=True

    else:
        res.correct=False
        print("\nSolution is not correct, try again")

    return res

def generate_hint():
    '''
    This is the funcion which creates the random hint. Hint are very random, it creates hint with the people, place and the weapon. And is ome random cases one of the element will be missed or more than than the three elements because of random generation of hints. It is called in the hint_req funcion in this module.        
    '''

    global people, weapons, places, num_ID_hint

    random.seed(datetime.now())

    #There's at least one hint from PERSON, PLACE and WEAPON
    num_hints= random.randint(1, 4)
    
    i=0
    what=[]
    where=[]
    who=[]
    ## The number of elements per hint is random
    while i!=num_hints:

        ## The type of element in the hint is random
        hint_type=random.randint(0,3)
        if hint_type==0:
            index_people= random.randint(0, len(people)-1) 
            who.append(people[index_people])

        elif hint_type==1:
            index_places= random.randint(0, len(places)-1)
            where.append(places[index_places])

        else:
            index_weapons= random.randint(0, len(weapons)-1)
            what.append(weapons[index_weapons])

        i=i+1
    
    # Update the number of hints generated 
    num_ID_hint+=1
    print("\nHint" + str(num_ID_hint) +": " + str(what) +", "+ str(where) + ", " + str(who))

    return [what, where, who]


def hint_req(req):
    '''
    This is the funciton which calls the generate hint function and send the generated hint to the state machine.         
    '''
    global num_ID_hint

    hint=generate_hint()

    res=AskHintResponse()
    res.what=hint[0]
    res.where=hint[1]
    res.who=hint[2]
    res.ID="HP" + str(num_ID_hint)
    return res




def main():
    '''
    This is the main of the oracle node which initializes the node. And creates the /solution and /hint_request topic. 
    '''

    global people, places, weapons

    # Initialize the node
    rospy.init_node('oracle')  

    rospy.Service('/solution', Solution, receive_solution)
    rospy.Service('/hint_request', AskHint, hint_req)

    # Generate a solution
    init_scene() 
    rospy.spin()



if __name__ == '__main__':
    main()
