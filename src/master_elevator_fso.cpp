#include "ros/ros.h"
#include "std_msgs/String.h"
#include "deque"
#include "simulation/calls.h"
#include <ctime>
#include <sys/time.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <algorithm>

/********************** DEFINE ************************/

#define dt 0.1 // [s]
#define ELEVATOR_TRAVEL_SPEED 0.5 // [floor/sec]
#define ELEVATOR_DWELLTIME_AT_FLOOR 3 // [sec]
#define NUMBER_OF_FLOORS 8 // [-]
#define TOP_FLOOR 7 // [-]
#define TERMINAL_FLOOR 0 // [-]

/********************** ENUM **************************/

enum{ UP = 1, DOWN = -1}; //Call direction
enum{ E1 = 1, E2 = 2}; //ElevatorID

/***************** GLOBAL VARIABLES *******************/

float current_time;
float totTravelTimeforPassengers = 0;
float totWaitingTimeforPassengers = 0;
float totNrPassengersHandled = 0;

/***************** ADJUSTABLE PARAMETER *******************/

//Decide how many people you want to handle.
//Ps. this must correpond with nr. of passengers for the poisson distribution
//in poisson_call_generator.cpp
float numberOfPassengersTravelling = 20; //[5 - 1000]

/************************ ELEVATOR CAR ****************************/
/*************************** CLASS ********************************/
// This class handles the elevator control of each separate elevator car

class ElevatorStatus{

public:

    //Status variables
    double exactElevatorPosition;
    unsigned int currentFloor;
    unsigned int destinationFloor;
    unsigned int assignedSector;
    unsigned int currentlyInSector;
    int travelDirection;
    bool idle;
    float totWaitingTime;
    float totTravelTime;
    float timeToNextDestinationFloor;

    //Constructor & initalizer
    ElevatorStatus();
    void initializeElevator();

    //Elevator handle algorithm
    int figureOfSuitability(int callAtFloor,int callGoingToFloor);

    //The elevator controller system
    void ElevatorStateController(int elevatorID);

    //Finds max and min floor in queue
    std::pair<unsigned int,unsigned int> getMaxAndMinFloorInQueue();

    //Elevator Ready Queues
    std::deque<std::pair<unsigned int,unsigned int> > pick_Up_Queue;
    std::deque<unsigned int> drop_Off_Queue;

};

/********************** THE MASTER ELEVATOR **************************/
/******************************* CLASS *******************************/
// This Class handles both elevators operating and includes the
// logical algorithm for choosing which elevator car that should handle
// a certain call.

class MasterElevator{

private:

    //Subscribers
    ros::Subscriber sub_elevator_call;
    ros::Subscriber sub_clock;

public:

    //Constructor
    MasterElevator(ros::NodeHandle &nh);

    //Fixed sectoring algorithm
    int fixedSectoring(int callAtFloor,int callGoingToFloor);

    //Class objects
    ElevatorStatus Elevator1;
    ElevatorStatus Elevator2;

    //Callback functions at 10Hz
    void getCurrentTime(const std_msgs::Float32::ConstPtr& subMsg);
    void pickElevatorToHandleCall(const simulation::calls& subMsg);

    //print elevators
    void print(int E1,int E2,int E1size,int E2size);
};

/************************ INITIALIZATION ****************************/


void ElevatorStatus::initializeElevator() {

    destinationFloor = 7;
    currentlyInSector = 1;
    travelDirection = UP;
    idle = true;
    totWaitingTime = 0;
    totTravelTime = 0;
}

/************************* CONSTRUCTORS *****************************/

//Constructing car object
ElevatorStatus::ElevatorStatus(){
    initializeElevator();
}


//Constructing Master Elevator object
MasterElevator::MasterElevator(ros::NodeHandle &nh) {

    //Creating two elevator car objects
    ElevatorStatus elev1;
    ElevatorStatus elev2;
    Elevator1 = elev1;
    Elevator2 = elev2;

    //Assign responsible sector and current floor
    Elevator1.assignedSector = 1;
    Elevator1.exactElevatorPosition = 0;
    Elevator1.currentFloor = 0;
    Elevator2.assignedSector = 2;
    Elevator2.exactElevatorPosition = 4;
    Elevator2.currentFloor = 4;


    //initialize subscribers
    sub_clock = nh.subscribe("/clock",1000,&MasterElevator::getCurrentTime,this);
    sub_elevator_call = nh.subscribe("/publishCallMsg",1000,&MasterElevator::pickElevatorToHandleCall,this);
}

/**************************** THE IMPLEMENTED ALGORITHM  ****************************/
/******************************* FSO FIXED SECTORING ********************************/


int MasterElevator::fixedSectoring(int callAtFloor,int callGoingToFloor) {

    //Declaration
    int callSector;
    //int directionOfCall;

    //Find the current section of elevators
    //Elevator 1
    if(Elevator1.currentFloor <= 3){
        Elevator1.currentlyInSector = 1;
    }else{
        Elevator1.currentlyInSector = 2;
    }

    //Elevator 2
    if(Elevator2.currentFloor > 3){
        Elevator2.currentlyInSector = 2;
    }else{
        Elevator2.currentlyInSector = 1;
    }

    //What sector is the call coming from
    if(callAtFloor <= 3){
        callSector = 1;
    }else{
        callSector = 2;
    }


    //The call is responsibility of sector 1 and elevator 1 is currently in that sector
    if(Elevator1.assignedSector == callSector) {
        if (Elevator1.currentlyInSector == Elevator1.assignedSector) {
            return E1;

            //if Elevator is currently not in its assigned sector
        } else if (Elevator1.currentlyInSector != Elevator1.assignedSector){

                //If elevator is travelling towards it sector
                if(Elevator1.travelDirection == DOWN){
                    return E1;}
                //if travelling away from its sector
                else{
                    return E2;}
        }

    //The call is responsibility of sector 2 and elevator is currently in that sector
    }else {
        if (Elevator2.currentlyInSector == Elevator2.assignedSector) {
            return E2;

            //if Elevator is currently not in its assigned sector
        } else if (Elevator2.currentlyInSector != Elevator2.assignedSector) {

            //If elevator is travelling towards it sector
            if (Elevator2.travelDirection == UP) {
                return E2;
                //if travelling away from its sector
            } else {
                return E1;
            }
        }
    }

    //if none of the above is right
    //then call is handled by the elevator responsible of that sector
    return callSector;
}

/*********************** FIND MAX & MIN FLOOR IN QUEUES ********************/

std::pair<unsigned int,unsigned int> ElevatorStatus::getMaxAndMinFloorInQueue() {

    //if the Queues are empty
    if (pick_Up_Queue.size() == 0 && drop_Off_Queue.size() == 0){
        return std::make_pair(0,0);
    }

    //This ensures that maxFloor = minFloor if only there is one
    //element left in both queues combined
    unsigned int maxFloor = 0;
    unsigned int minFloor = 7;

    //Check pick-up Queue first if not empty
    if(pick_Up_Queue.size() > 0) {
        for (std::deque<std::pair<unsigned int, unsigned int> >::iterator it = pick_Up_Queue.begin(); it != pick_Up_Queue.end(); ++it) {
            maxFloor = std::max(it->first, maxFloor);
            minFloor = std::min(it->first, maxFloor);
        }
    }

    //Check drop-off Queue second if not empty
    if(drop_Off_Queue.size() > 0) {
        for (std::deque<unsigned int>::iterator it = drop_Off_Queue.begin(); it != drop_Off_Queue.end(); ++it) {

            maxFloor = std::max(*it, maxFloor);
            minFloor = std::min(*it, minFloor);
        }
    }

        return std::make_pair(maxFloor,minFloor);
}




/************************ ELEVATOR CONTROLLER *************************/
// This function handles the actual real-time control of the elevators
// It handles both pick-up & drop-off of passengers, sets new destinations,
// handles travelling between floors
void ElevatorStatus::ElevatorStateController(int elevatorID) {

    //Calculate elevator position
    //If there is any request for the elevator, then move
    //If not, stay idle
    if (pick_Up_Queue.size() > 0 || drop_Off_Queue.size() > 0) {
        idle = false;
        exactElevatorPosition += travelDirection * (ELEVATOR_TRAVEL_SPEED * dt);
        currentFloor = floor(exactElevatorPosition);

        //Check if current floor is in the pick up Queue

        if (pick_Up_Queue.size() > 0) {

            //Add up total waiting time
            totWaitingTimeforPassengers += pick_Up_Queue.size() * dt;

            //Check if current floor is queued
            for (std::deque<std::pair<unsigned int, unsigned int> >::iterator it = pick_Up_Queue.begin();
                 it < pick_Up_Queue.end(); it++) {

                if (it->first == currentFloor) {
                    //If current floor is in the pick up Queue add to drop off Queue
                    //and then pop element from pick up Queue
                    drop_Off_Queue.push_back(it->second);

                    //Break out of for-loop if the queue is empty
                    if (pick_Up_Queue.size() > 0) {
                        pick_Up_Queue.erase(it);
                    }else{
                        break; //leave for-loop
                    }

                    //Also add dwelling time for stopping at floor
                    totWaitingTime += ELEVATOR_DWELLTIME_AT_FLOOR * dt;

                }//if

            }//for
        }//if

        //check if current floor is in the drop off Queue
        if (drop_Off_Queue.size() > 0) {

            //Add up total travel times
            totTravelTimeforPassengers += drop_Off_Queue.size() * dt;

            //Check if current floor is queued
            for (std::deque<unsigned int>::iterator it = drop_Off_Queue.begin(); it < drop_Off_Queue.end(); it++) {
                if (drop_Off_Queue.size() <= 0) {
                    break; //leave for loop

                }

                if (*it == currentFloor) {

                    //Visit is registered by removing element from queue
                    if(drop_Off_Queue.size()>0) {
                        drop_Off_Queue.erase(it);
                    }
                } //if
            } //for
        } //if


        /******************** CHOOSING NEXT DESTINATION FLOOR *******************/
        //Here we want to set: travelDirection and destination floor

        unsigned int maxFloor = getMaxAndMinFloorInQueue().first;
        unsigned int minFloor = getMaxAndMinFloorInQueue().second;


        //Upward riding logic
        if ((maxFloor > currentFloor) && travelDirection == UP) {
            travelDirection = UP;
            destinationFloor = maxFloor;
        } else if (currentFloor == TERMINAL_FLOOR) {
            travelDirection = UP;
            destinationFloor = maxFloor;
        }

        //Downward riding logic
        if (maxFloor <= currentFloor) {
            travelDirection = DOWN;
            destinationFloor = minFloor;
        } else if ((minFloor < currentFloor) && travelDirection == DOWN) {
            travelDirection = DOWN;
            destinationFloor = minFloor;
        } else if (currentFloor == TOP_FLOOR) {
            travelDirection = DOWN;
            destinationFloor = minFloor;
        }


        //If elevator has no calls in queue
        //Then it is idle
    }else{
        idle = true;
    }

 }

/******************************* SIMULATION ********************************/
/*************************** CALLBACK FUNCTIONS ****************************/

void MasterElevator::print(int E1, int E2,int E1size, int E2size) {

    for (int floor = 7; floor >= 0; floor--) {

        if(floor == 7) {
            std::cout << " ***********************" << std::endl;
            std::cout << " **** FSO ALGORITHM ****" << std::endl;
            std::cout << " ***********************" << std::endl;
        }

        if (E1 != floor && E2 != floor){

            std::cout << "_________________________" << std::endl;
            std::cout << "|           |            |" << std::endl;
            std::cout << "|           |            |" << std::endl;
            std::cout << "|           |            |" << std::endl;
            std::cout << "|" <<floor<<"          |            |" << std::endl;

        }else if(E1 == floor && E2 != floor){
            std::cout << "_________________________" << std::endl;
            std::cout << "|    ___    |            |" << std::endl;
            std::cout << "|   | "<<E1size<<" |   |            |" << std::endl;
            std::cout << "|   |___|   |            |" << std::endl;
            std::cout << "|" <<floor<<"          |            |" << std::endl;
        }else if(E1 != floor && E2 == floor){
            std::cout << "_________________________" << std::endl;
            std::cout << "|           |     ___    |" << std::endl;
            std::cout << "|           |    | "<<E2size<<" |   |" << std::endl;
            std::cout << "|           |    |___|   |" << std::endl;
            std::cout << "|" <<floor<<"          |            |" << std::endl;
        }else{
            std::cout << "_________________________" << std::endl;
            std::cout << "|    ___    |     ___    |" << std::endl;
            std::cout << "|   | "<<E1size<<" |   |    | "<<E2size<<" |   |" << std::endl;
            std::cout << "|   |___|   |    |___|   |" << std::endl;
            std::cout << "|" <<floor<<"          |            |" << std::endl;
        }
    }

}

//Updates the global timing
void MasterElevator::getCurrentTime(const std_msgs::Float32::ConstPtr& subMsg){
    current_time = subMsg->data;
}

//Handling of calls
void MasterElevator::pickElevatorToHandleCall(const simulation::calls& subMsg) {

    //New call
    bool newCall = subMsg.newCall;
    float time = subMsg.time;
    unsigned int floor = subMsg.floor;
    int direction = subMsg.direction;

    if (newCall){

        //Update total number of passengers
        totNrPassengersHandled++;

        //make queue pair
        std::pair<unsigned int,unsigned int> call;
        call = std::make_pair(floor, direction);

        //Find elevator responsible for this sector
        int elevatorID = fixedSectoring(floor,direction);

        //Elevator responsible of requested sector gets the call
        if (elevatorID == E1){
            Elevator1.pick_Up_Queue.push_back(call);
        }else{
            Elevator2.pick_Up_Queue.push_back(call);
        }
    }//if

    /************************** Print *****************************/

    //Update the state of the elevators;
    Elevator1.ElevatorStateController(E1);
    Elevator2.ElevatorStateController(E2);
    print(Elevator1.currentFloor,Elevator2.currentFloor,
          Elevator1.drop_Off_Queue.size(),Elevator2.drop_Off_Queue.size());

    /************************** SIMULATION COMPLETES AT *****************************/

    if(totNrPassengersHandled == numberOfPassengersTravelling && Elevator1.idle && Elevator2.idle){
        std::cout << "\n \n SIMULATION IS COMPLETE! \n \n" << std::endl;
        std::cout << "STATISTICS: \n" << std::endl;

        std::cout << "Clock at completion: " << current_time << std::endl;
        std::cout << "Total number of passengers handled: " << totNrPassengersHandled << std::endl;
        std::cout << "Total waiting time for all passengers combined: " << totWaitingTimeforPassengers << std::endl;
        std::cout << "Total travel time for all passengers combined: " << totTravelTimeforPassengers << std::endl;
        std::cout << "Total response time for all passengers combined: " << (totTravelTimeforPassengers + totWaitingTimeforPassengers) << std::endl;
        std::cout << "\n \n";

        std::cout << "Average travel time: " << totTravelTimeforPassengers/totNrPassengersHandled << std::endl;
        std::cout << "Average response time: " << (totTravelTimeforPassengers + totWaitingTimeforPassengers)/totNrPassengersHandled << std::endl;
        std::cout << "RESULT OF OBJECTIVE FUNCTION: \n \n";
        std::cout << "Average waiting time: " << totWaitingTimeforPassengers/totNrPassengersHandled << std::endl;


        ros::shutdown();
        //to only print once
    }
}



/******************************* ROS NODEHANDLE *******************************/

int main(int argc, char** argv){
    ros::init(argc, argv, "master_elevator_fso");
    ros::NodeHandle nh;
    MasterElevator M(nh);

    ros::spin();
    return 0;
}
