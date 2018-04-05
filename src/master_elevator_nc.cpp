#include "ros/ros.h"
#include "std_msgs/String.h"
#include "armadillo"
#include "queue"
#include "simulation/calls.h"
#include <ctime>
#include <sys/time.h>
#include <std_msgs/Float32.h>

// ********************** DEFINE *********************** //
#define dt 0.1 // [s]
#define ELEVATOR_TRAVEL_SPEED 2 // [floor/sec]
#define ELEVATOR_DOOR_CLOSING_TIME 1 // [sec]
#define TOP_FLOOR 7 // [-]
#define TERMINAL_FLOOR 0 // [-]
enum{ UP = 1, DOWN = -1};




class ElevatorStatus{

//private:
public:
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

    ElevatorStatus();
    void initializeElevator();
    void setElevatorStatus();
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

        std::cout << "new call: " << std::endl;




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