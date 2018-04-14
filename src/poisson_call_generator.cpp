#include "ros/ros.h"
#include "std_msgs/String.h"
#include "armadillo"
#include "queue"
#include "simulation/calls.h"
#include <ctime>
#include <sys/time.h>
#include <std_msgs/Float32.h>
#include <random>

/********************** DEFINE ************************/
#define dt 0.1
/********************** ENUM **************************/
enum{TERMINAL = 0, TOP = 7};
/***************** GLOBAL VARIABLES *******************/
float current_time;
int count = 0;
/***************************** CALL OBJECT **********************************/

class Call{
public:

    //Call variables
    double time;
    unsigned int floor;
    int direction;

    //Constructor
    Call(float t, int flr, int dir){
        time = t;
        floor = flr;
        direction = dir;
    };

    //Destructor
    ~Call(){};
};

/************************ PASSENGER CALL GENERATOR ************************/
// Has the responsiblility of constructing a queue of passenger calls
// that are all
class CallGenerator{
private:

    //Publishing topics
    ros::Publisher pub_elevator_call;
    ros::Publisher pub_clock;


public:

    //Ros node constructor
    CallGenerator(ros::NodeHandle &nh);

    //Queue to store all calls
    std::queue<Call> requests;

    //Time
    std::time_t tstart, tstop;
    void time_to_publish();

    //Necessary for specifing the running frequency
    void spin();	

};


/*********************** PASSENGER FLOW POISSON PROCESS **************************/
//Generating arrival times via Poisson Process by creating an
//exponential distribution based on the Poisson arrival rate lamda.

CallGenerator::CallGenerator(ros::NodeHandle &nh){

    // seed the RNG
    std::random_device rd; // uniformly-distributed integer random number generator
    std::mt19937 rng (rd ()); // mt19937: Pseudo-random number generation

    int totNumberOfPassengers = 20;
    double averageArrivalTime = 2;
    double lamda = 1 / averageArrivalTime;
    std::exponential_distribution<double> exp (lamda);

    double sumArrivalTimes=0;
    double newArrivalTime;
    int floor;
    int direction;

    for (int i = 0; i < totNumberOfPassengers; i++)
    {
        /* GENERATING ARRIVAL TIMES VIA POISSON PROCESS */
        newArrivalTime=  exp.operator() (rng);// generates the next random number in the distribution
        sumArrivalTimes  = sumArrivalTimes + newArrivalTime;

        //Generate random floor in range [0 - 7]
        floor = ( std::rand() % (  TOP - TERMINAL + 1 ) );

        //Generate random floor in range [0 - 7], but different from "floor"
        do {
            direction = (std::rand() % (TOP - TERMINAL + 1));
        }while(floor == direction);

        //Store the call in a queue
        Call call(sumArrivalTimes,floor,direction);
        requests.push(call);

    }

    //Publishers
    //1000 is the message buffer
    pub_elevator_call = nh.advertise<simulation::calls>("/publishCallMsg",1000);
    pub_clock = nh.advertise<std_msgs::Float32>("/clock",1000);

    //Set start time
    tstart = std::time(0);

    std::cout << "PASSENGER FLOW HAS STARTED!" << std::endl;
    std::cout << "waiting for calls to arrive..." << std::endl;
}


void CallGenerator::time_to_publish() {

    //Create msg object
    simulation::calls msg;

    //Initiate new calls as false
    msg.newCall = false;

    if (requests.size() > 0) {

        //Extract next element in queue
        Call latest_call = requests.front();

        //if new call has arrived
        if (current_time > latest_call.time) {

            msg.newCall = true;
            msg.time = latest_call.time;
            msg.floor = latest_call.floor;
            msg.direction = latest_call.direction;

            //write to screen
            std::cout << " \n \n ********* NEW PASSENGER REQUESTING ELEVATOR **********\n " << std::endl;

            std::cout << "Time of call: " << requests.front().time << std::endl;
            std::cout << "From floor: " << requests.front().floor << std::endl;
            std::cout << "Going to floor: " << requests.front().direction << std::endl;
            std::cout << " \n \n CALL IS SENT TO MASTER ELEVATOR! " << std::endl;

            //remove caller from queue
            requests.pop();

        }
    }else{
        if(count == 0) {
            std::cout << "POISSON DISTRIBUTED PASSENGER FLOW IS COMPLETE!" << std::endl;
            std::cout << "Waiting for elevators to complete their simulation..." << std::endl;

            count++;
        }
    }

    //publish
    pub_elevator_call.publish(msg);
}



/******************************* ROS NODEHANDLE *******************************/

void CallGenerator::spin(){
    ros::Rate loop_rate(1.0 / dt);
    while (ros::ok())
    {

        //Publishing the time
        tstop= std::time(0);
        std_msgs::Float32 clock;
        current_time = (tstop-tstart);
        clock.data = current_time;
        pub_clock.publish(clock);

        this->time_to_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {

    /*Initializing ROS system*/
    ros::init(argc, argv, "poisson_call_generator");
    ros::NodeHandle nh;

    CallGenerator *call;
    CallGenerator temp_call(nh);
    call = &temp_call;
    call->spin();
    return 0;
}
