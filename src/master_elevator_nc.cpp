#include "ros/ros.h"
#include "std_msgs/String.h"
#include "armadillo"
#include "deque"
#include "simulation/calls.h"
#include <ctime>
#include <sys/time.h>
#include <std_msgs/Float32.h>
#include <math.h>

// ********************** DEFINE *********************** //
#define dt 0.1 // [s]
#define ELEVATOR_TRAVEL_SPEED 1.0 // [floor/sec]
#define ELEVATOR_DWELLTIME_AT_FLOOR 3 // [sec]
#define NUMBER_OF_FLOORS // [-]
#define TOP_FLOOR 7 // [-]
#define TERMINAL_FLOOR 0 // [-]
enum{ UP = 1, DOWN = -1};




class ElevatorStatus{

//private:
public:

    //Status variables
    double exactElevatorPosition;
    unsigned int currentFloor;
    unsigned int destinationFloor;
    unsigned int nrPassengersInElevator;
    unsigned int totNrPassengersHandled;
    int travelDirection;
    //bool idle;
    //bool empty;
    float totWaitingTime;
    float totTravelTime;
    float timeToNextDestinationFloor;

    //Constructor & initalizer
    ElevatorStatus();
    void initializeElevator();

    //Elevator handle algorithm
    int figureOfSuitability(int callAtFloor,int callGoingToFloor);

    //The elevator controller system
    void ElevatorStateController();

    //Elevator Queues
    //std::deque<unsigned int> pick_Up_Queue;
    std::deque<std::pair<unsigned int,unsigned int> > pick_Up_Queue;
    std::deque<unsigned int> drop_Off_Queue;

};

class MasterElevator{

private:

    //Subscribers
    ros::Subscriber sub_elevator_call;
    ros::Subscriber sub_clock;

    float timeLastCall;
    unsigned floorLastCall;
    int directionLastCall;

public:

    int totNrPassengersHandled;

    //MasterElevator(){timeLastCall = 0; floorLastCall = 0;directionLastCall = 1;}
    MasterElevator(ros::NodeHandle &nh);
    ElevatorStatus Elevator1;
    ElevatorStatus Elevator2;

    //callback
    void getCurrentTime(const std_msgs::Float32::ConstPtr& subMsg);
    void pickElevatorToHandleCall(const simulation::calls& subMsg);
};

// ************************ INITIALIZATION **************************** //

// ********************** Global ************************ //

//global time
float current_time;

//create two elevator objects
//ElevatorStatus Elevator1;
//ElevatorStatus Elevator2;
//MasterElevator Master;

void ElevatorStatus::initializeElevator() {
    exactElevatorPosition = 0;
    currentFloor = 0;
    nrPassengersInElevator = 0;
    //totNrPassengersHandled = 0;
    travelDirection = UP;
    //idle = true;
    //empty = true;
    totWaitingTime = 0;
    totTravelTime = 0;
}

// Constructor
ElevatorStatus::ElevatorStatus(){
    initializeElevator();
}

// ******************************************************************** //

//Constructor

MasterElevator::MasterElevator(ros::NodeHandle &nh) {

    ElevatorStatus elev1;
    ElevatorStatus elev2;
    Elevator1 = elev1;
    Elevator2 = elev2;

    timeLastCall = 0;
    floorLastCall = 0;
    directionLastCall = 1;

    totNrPassengersHandled = 0;

    //initialize subscribers
    sub_clock = nh.subscribe("/clock",1000,&MasterElevator::getCurrentTime,this);
    sub_elevator_call = nh.subscribe("/publishCallMsg",1000,&MasterElevator::pickElevatorToHandleCall,this);
}

// ************************* ELEVATOR HANDLE LOGIC ******************


int ElevatorStatus::figureOfSuitability(int callAtFloor,int callGoingToFloor) {

    //Figure of suitability
    int FS;

    int directionOfCall = callGoingToFloor - callAtFloor;
    int currentMovingDirection = destinationFloor - currentFloor;
    int distanceToCaller = callAtFloor - currentFloor;
    distanceToCaller = abs(distanceToCaller);

    std::cout << "directionOfCall: " << directionOfCall << std::endl;
    std::cout << "currentMovingDirection: " << currentMovingDirection << std::endl;
    std::cout << "distanceToCaller: " << distanceToCaller << std::endl;
    std::cout << "currentFloor: " << currentFloor << std::endl;

    if ((currentMovingDirection > 0 && directionOfCall > 0) || (currentMovingDirection < 0 && directionOfCall < 0)){
        //the elevator car is moving towards the landing call and the call is set in the same direction.
        FS = NUMBER_OF_FLOORS + 1 - (distanceToCaller-1);
        std::cout << "FS1: ";
    }else if((currentMovingDirection < 0 && directionOfCall > 0) || (currentMovingDirection > 0 && directionOfCall < 0) ){
        //the elevator car is moving towards the landing call but the call is set to the opposite direction.
        FS = NUMBER_OF_FLOORS + 1 - distanceToCaller;
        std::cout << "FS2: ";
    }else if(((currentFloor > callAtFloor) && currentMovingDirection > 0)||((currentFloor < callAtFloor) && currentMovingDirection < 0)){
        //the elevator car is already moving away from the landing call (the elevator is responding to some other call).
        FS = 1;
       std::cout << "FS3: ";
    }else{
        FS = NUMBER_OF_FLOORS+1 - distanceToCaller;
        std::cout << "FS4: ";
    }
    std::cout << std::endl;
    //std::cout << FS << std::endl;

    return FS;
}




// ********************* ELEVATOR CONTROLLER ******************************


void ElevatorStatus::ElevatorStateController() {

    //Calculate elevator position

    //If there is any request for the elevator, then move
    if (pick_Up_Queue.size() > 0 || drop_Off_Queue.size() > 0) {
        exactElevatorPosition += travelDirection * (ELEVATOR_TRAVEL_SPEED * dt);
        currentFloor = floor(exactElevatorPosition);
        std::cout << "ExactElevatorPosition: " << exactElevatorPosition << std::endl;
    }

    destinationFloor = 7;

    if (currentFloor == 7){
        destinationFloor = 1;
    }

    //Check if current floor is in the pick up Queue

    if (pick_Up_Queue.size() > 0) {

        //Add up total waiting time
        totWaitingTime += pick_Up_Queue.size()*dt;


        //Check if current floor is queued
        for (std::deque<std::pair<unsigned int,unsigned int> >::iterator it=pick_Up_Queue.begin(); it!=pick_Up_Queue.end(); ++it) {

            std::cout << "it->first: " << it->first << "it->second: " << it->second << std::endl;
            std::cout << "pick up size: " << pick_Up_Queue.size() << "drop off size: " << drop_Off_Queue.size() << std::endl;

            if (it->first == currentFloor){
                //If current floor is in the pick up Queue add to drop off Queue
                //and then pop element from pick up Queue
                drop_Off_Queue.push_back(it->second);
                pick_Up_Queue.erase(it);

                //Also add dwelling time for stopping at floor
                totWaitingTime += ELEVATOR_DWELLTIME_AT_FLOOR*dt;
                std::cout << "it->first: " << it->first << "it->second: " << it->second << std::endl;
                std::cout << "totTime: " << totWaitingTime << std::endl;
                std::cout << "Elevator picks up passenger at floor: " << currentFloor << std::endl;

                break; //leave for loop

            }//if
        }//for
        std::cout << "out of for loop " << std::endl;
    }//if

    //check if current floor is in the drop off Queue
    if (drop_Off_Queue.size() > 0){
        std::cout << "inside drop off" << std::endl;
        //Add up total travel times
        totTravelTime += drop_Off_Queue.size()*dt;

        //Check if current floor is queued
        for (std::deque<unsigned int>::iterator it=drop_Off_Queue.begin(); it!=drop_Off_Queue.end(); ++it){

            if(*it == currentFloor){

                //Visit is registered by removing element from queue
                drop_Off_Queue.erase(it);

                std::cout << "Elevator drops of passenger at floor" << currentFloor << std::endl;
            }
            std::cout << "inside for" << std::endl;
        }


    }

        /*
        //elevator is travelling UP
        if(travelDirection == UP){

            int tempNextDestinationFloor;
            //Find all queued floors in this direction
            for (it=mydeque.begin(); it!=mydeque.end(); ++it) {
                std::cout << ' ' << *it;

                if(*it > currentFloor){
                    tempNextDestinationFloor = *it;
                }//if
             }//for



        }//if
*/


std::cout << "end of callback" << std::endl;


 }







//callback function
void MasterElevator::getCurrentTime(const std_msgs::Float32::ConstPtr& subMsg){
    current_time = subMsg->data;
}

//callback function
void MasterElevator::pickElevatorToHandleCall(const simulation::calls& subMsg) {
std::cout << "pickelevatorCount" << std::endl;
    //New call
    bool newCall = subMsg.newCall;
    float time = subMsg.time;
    unsigned int floor = subMsg.floor;
    int direction = subMsg.direction;

    //std::cout << "Direction: " << direction << std::endl;

    if (newCall){
    std::cout << "newcall" << std::endl;
        //Update total number of passengers
        totNrPassengersHandled++;

        //Calculate Figure of Suitability for both elevatorhs
        int FS_E1 = Elevator1.figureOfSuitability(0,direction);
        int FS_E2 = Elevator2.figureOfSuitability(3,direction);

        std::cout << "FS_E1: " << FS_E1 << std::endl;
        std::cout << "FS_E2: " << FS_E2 << std::endl;

        //make queue pair
        std::pair<unsigned int,unsigned int> call;
        call = std::make_pair(floor, direction);


        //Largest Figure of suitability get the call
        if (FS_E1 >= FS_E2){
            Elevator1.pick_Up_Queue.push_back(call);
            //Elevator1.drop_Off_Queue.push_back(direction);
        }else{
            Elevator2.pick_Up_Queue.push_back(call);
            //Elevator2.drop_Off_Queue.push_back(direction);
        }

    }//if

    //Update the state of the elevators;
    Elevator1.ElevatorStateController();
    Elevator2.ElevatorStateController();
}


// ******************************* ROS specific*******************************//

int main(int argc, char** argv){
    ros::init(argc, argv, "master_elevator_nc");
    ros::NodeHandle nh;
    MasterElevator M(nh);

    ros::spin();
    return 0;
}