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
HHJ

