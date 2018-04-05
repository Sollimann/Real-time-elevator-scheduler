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


float current_time;



class MasterElevator{

private:

    //Subscribers
    ros::Subscriber sub_elevator_call;
    ros::Subscriber sub_clock;

    //private variables


public:

    MasterElevator(ros::NodeHandle &nh);

    //callback
    void getCurrentTime(const std_msgs::Float32::ConstPtr& subMsg);
    void pickElevatorToHandleCall(const simulation::calls& subMsg);
};


class ElevatorStatus{

private:

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

public:

    void initializeElevator();

};

// ************************ INITIALIZATION ****************************//

void ElevatorStatus::initializeElevator() {




}


//Constructor
MasterElevator::MasterElevator(ros::NodeHandle &nh) {



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
    time = subMsg.time;
    floor = subMsg.floor;
    direction = subMsg.direction;



}


// ******************************* ROS specific*******************************//

int main(int argc, char** argv){
    ros::init(argc, argv, "node_distance_logger");
    ros::NodeHandle nh;
    MasterElevator master(nh);
    ros::spin();
    return 0;
}