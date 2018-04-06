#include "ros/ros.h"
#include "std_msgs/String.h"
#include "armadillo"
#include "queue"
#include "simulation/calls.h"
#include <ctime>
#include <sys/time.h>
#include <std_msgs/Float32.h>
#include <math.h>

// ********************** DEFINE *********************** //
#define dt 0.1 // [s]
#define ELEVATOR_TRAVEL_SPEED 1 // [floor/sec]
#define ELEVATOR_DWELLTIME_AT_FLOOR 3 // [sec]
#define NUMBER_OF_FLOORS // [-]
#define TOP_FLOOR 7 // [-]
#define TERMINAL_FLOOR 0 // [-]
enum{ UP = 1, DOWN = -1};




class ElevatorStatus{

//private:
public:

    //Status variables
    unsigned int currentFloor;
    unsigned int destinationFloor;
    unsigned int nrPassengersInElevator;
    unsigned int totNrPassengersHandled;
    int travelDirection;
    bool idle;
    bool empty;
    float totWaitingTime;
    float totTravelTime;
    float timeToNextDestinationFloor;

    //Constructor & initalizer
    ElevatorStatus();
    void initializeElevator();

    //Elevator handle algorithm
    int figureOfSuitability(unsigned int callAtFloor,unsigned int callGoingToFloor);

    //The elevator controller system
    void ElevatorStateController();

    //Elevator Queues
    std::queue<unsigned int> pick_Up_Queue;
    std::queue<unsigned int> pick_up_Drop_Off_Queue;

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
    currentFloor = 0;
    nrPassengersInElevator = 0;
    totNrPassengersHandled = 0;
    travelDirection = UP;
    idle = true;
    empty = true;
    totWaitingTime = 0;
    totTravelTime = 0;
}

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

    //initialize subscribers
    sub_clock = nh.subscribe("/clock",1000,&MasterElevator::getCurrentTime,this);
    sub_elevator_call = nh.subscribe("/publishCallMsg",1000,&MasterElevator::pickElevatorToHandleCall,this);
}

// ************************* ELEVATOR HANDLE LOGIC ******************


int ElevatorStatus::figureOfSuitability(unsigned int callAtFloor,unsigned int callGoingToFloor) {

    //Figure of suitability
    int FS;

    int directionOfCall = callGoingToFloor - callAtFloor;
    int currentMovingDirection = destinationFloor - currentFloor;
    int distanceToCaller = std::fabs(callAtFloor - currentFloor);


    if ((currentMovingDirection > 0 && directionOfCall > 0) || (currentMovingDirection < 0 && directionOfCall < 0)){
        //the elevator car is moving towards the landing call and the call is set in the same direction.
        FS = NUMBER_OF_FLOORS + 1 - (distanceToCaller-1);
        std::cout << "FS1: ";
    }
    else if((currentMovingDirection < 0 && directionOfCall > 0) || (currentMovingDirection > 0 && directionOfCall < 0) ){
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

    std::cout << FS << std::endl;

    return FS;
}




// ********************* ELEVATOR CONTROLLER ******************************


void ElevatorStatus::ElevatorStateController() {







}







//callback function
void MasterElevator::getCurrentTime(const std_msgs::Float32::ConstPtr& subMsg){
    current_time = subMsg->data;
}

//callback function
void MasterElevator::pickElevatorToHandleCall(const simulation::calls& subMsg) {

    //New call
    bool newCall = subMsg.newCall;
    float time = subMsg.time;
    unsigned int floor = subMsg.floor;
    int direction = subMsg.direction;

    if (newCall){






    }//if
}


// ******************************* ROS specific*******************************//

int main(int argc, char** argv){
    ros::init(argc, argv, "node_distance_logger");
    ros::NodeHandle nh;
    MasterElevator M(nh);

    ros::spin();
    return 0;
}