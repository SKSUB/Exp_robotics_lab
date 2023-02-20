# Exp_robotics_lab Assignment1
## INTROCUCTION
Purpose of this repository is to develop the assignment 1 of Experimental Robotics Laboratory. The assignment is about developing the simulation of the cluedo game in the ros environment. Cluedo is a board game, the objective of the game is to determine who murdered the game's victim, where the crime took place, and which weapon was used(more details on https://en.wikipedia.org/wiki/Cluedo). In this simulation robot moves between the rooms as in a cluedo game to look for the hints and once it find the hints it goes back to the oracle room to check the solution. If the solution is correct the game ends or the robot look for more hints by moving around the rooms. At first the solution is generated, then the robot moves randomly between places and finds the hints. Then it checks for the consistency with the Armor service, once the generated is conistent. It checks for the correct solution. Games ends based when the solution is correct.

## SOFTWARE ARCHITECTURE
In order to better understand the software architecture we can utilise the UML component diagram

UML COMPONENT DIAGRAM
![CLUDO_FSM](https://user-images.githubusercontent.com/82164428/219979474-29c5449c-e85b-4ddb-817e-4d44d3e8b080.jpg)

Finite state machine node is the main node of the architecure, with which each and every node communicates. It simulates the scene of the cluedo gmae. At first orcale node genreates the solution when the game starts. 

Finite state machine node ask the go to point node to simulate the movement of the robot from point to point. It sends the co ordinates to the go to point to simulate the movibg action.

It ask the hints with the oracle node. Oracel node provide the hint (what[], where[], who[]) with the ID. It also ask the oracle node for solution check. Oracle send back the true or false for the solution check.

It communicates with the myArmor class to effectively communicate with the armor service. myArmor class communicates with the armor service and sends the response from armor service back to the state machine node. It perfoems all the armor functions. 

It communicates with the place class to generate the place object with co ordinates of the place and its name. 

Lets dive into the details of the software architecture of the ros workspace:
The workspace consist of the scripts , launch and srv directories & cluedo ontology file. Scripts directory contains all the scripts and class to execure the architecture. Launch folder contain the launch file to execute all the nodes to simulate the game. srv folder has all the service messages. 

### SCRIPTS:

### STATE_MACHINE.PY: 
This node stimulate the three sates of the robot. The three states are move state, enter the room state and the solution check state. 
  
 1)MOVE: In the first state the robot starts from the oracle room and start to explore rooms in order to acquire hints. The Move custom service message is send to the go_to_point node in order to simulate the movement of the robot. And it is followed by any one of the following state.
 2)ENTER_ROOM: The robot enters the room and ask the hint with the AskHint custom message to oracle node, oracle node provides the hint with the ID. Then it add the hints to the ontology by MYArmor class and checks for the consistency. If it is consistent, it goes to the oracle room to check the solution.
 3)SOLUTION: This is the state which the robot try to check the solution with the Solution custom message to the oracle node. Based on the solution i.e whether it is true or false it ask to end of game solution or goes back to move node.
 
 ### ORACLE.PY
 ORACLE is the simulator of the oracle room of the game. It generates the solution at first. Whenver the hint is requested it genrates the random hints. Hints are always random with the place, person and weapon and it is generated with the hint ID. Hints are too random sometimes inconsistent and incomplete hints are generated. It also checks for solution correctness when requested by the state machine node.
 
 ### go_to_point.py:
 This is a node simulate the movement of the robot. It generates wait time when the state machine request for the movement simulation. It calculates the euclidean diestance between the point one to another and wait for the time to reach the distance. And sends the complete signal to state machine.
 
 ### myArmor class:
 This is the class method to communicate with the Armor service. It is based on the message format to be communicated with the Armor service.
 
 ### place class:
 THis is the class to create a place object.
 
 ### LAUNCH FILE: simulation.launch
 Launch file for launching all the above nodes as well as smach viewer.
 
 ### service messages:
 AskHint.srv for asking hint with the ID.
 
 Move.srv for sending the position of the starting and ending place. Returns bool message.
 
 AskHint.srv sends the hint and returns bool message.
 
 The cleudo onotology file contains the knowledge of the initial scene. Once the solution is obtianed solution_cluedo_ontology file is generated with the solution hypothesis. 
 
 ### OTHER PACKAGES:
 ARMOR SERVICE (More details in https://github.com/EmaroLab/armor)
 
 SMACH (More details in https://wiki.ros.org/smach)
 
 You need to have this packages in your workspace to run the simulation
 
 In order to better visualise the order of the simulation, a temporal diagram is created.
 
 TEMPORAL DIAGRAM:
 
 ![cluedo_temporal](https://user-images.githubusercontent.com/82164428/219983056-8846a53e-a141-4303-9bd0-0469e2532b10.jpg)
 
 ## INSTALLATION AND RUNNING:
 
 In order to run the simulation clone this repostitory into your workspace. If you are running on the docker machine of the course no need to clone other packages. Otherwise clone the smach and armor service packages into your workspace. And more details on installation and running armor and smach on the link provided above.
 ```
 git clone https://github.com/SKSUB/Exp_robotics_lab_1
 ```

After cloning usual procedures of building the package, and install the dependencies based on the error.
```
catkin_make
```
Run the roscore and armor service before running the simulation.
```
roscore
```
```
rosrun armor execute it.emarolab.armor.ARMORMainService
```
Then launch the simulation with the launch file
```
roslaunch Exp_robotics_lab_1 simulation.launch
```
This will launch all the nodes and smach viewer.

Following are the pictures showing the result of simulation:

Running simulation:

![running](https://user-images.githubusercontent.com/82164428/219983786-a07a1c18-617b-45d5-87ae-3b0306340cde.png)

The above pictures shows the smach viewer with all the running node. You can really view all the states of the robot in the smach viewer.

End result:

![Screenshot 2023-02-19 224952](https://user-images.githubusercontent.com/82164428/219983863-542313ac-c69d-4e7b-8704-ebd087152d3d.png)

RQT graph:

![rosgraph](https://user-images.githubusercontent.com/82164428/219989710-66fd17a1-7deb-47dd-b993-f9ef9c1dde45.png)

This is the obtained solution at the end of the simulation.

## SYSTEM WORKING AND LIMITATIONS
The working of the system is to generate the hints and perform communication with the armor service, the loop is continued until the hint gnerated need to be complete and consistent. Once the generated hint is complete and consistent, it is check for solution. Again the loop is continued from the first step. In the between stages, the robot has to move between places randomly and collect all the hints. And also all the hints need to be added to ontology and removed from the ontology if they are not complete and incosistent.

So based on the above working hypothesis the straightforward limitation is the simulation is time consumimg because of all the randombness attach with it. Each time the random hints are added and removed. And there is wait time for the robot to move between places. The hints generation randomness is time consumign process, it takes more time to generate the complete and consistent hint. And that one as well need to be a solution. And every armor service is called to performed to query the ontology. Even though sometimes it is called for the inconsistent one. And also the inconsistent one are added and removed.

In the future to improve the system, the hints genration are first check for completeness at first and then the armor service is called. Then we can add them ontology based on armor response. 

### CONTACTS
SATHISH KUMAR SUBRAMANI.
ROBOTICS ENGINEER, UNIGE.
4847560@studenti.unige.it
            

