#include "ros/ros.h"
#include "std_msgs/String.h"
#include "armadillo"
#include "queue"
#include "simulation/calls.h"
#include <ctime>
#include <sys/time.h>
#include <std_msgs/Float32.h>



#define dt 0.1

//Direction of call
enum{ UP = 1, DOWN = -1};
enum{TERMINAL = 0, TOP = 7};
float current_time;

class Call{
public:
    double time;
    unsigned int floor;
    int direction;

    //Constructor
    Call(float t, int flr, int dir){
        time = t;
        floor = flr;
        direction = dir;
    };

    //Deconstructor
    ~Call(){/*std::cout << "Destructor! " << std::endl;*/};
};


class CallGenerator{
private:
    ros::Publisher pub_elevator_call;
    ros::Publisher pub_clock;


public:
    CallGenerator(ros::NodeHandle &nh);
    std::queue<Call> requests;
    std::time_t tstart, tstop;
    void time_to_publish();

    void spin();	

};


CallGenerator::CallGenerator(ros::NodeHandle &nh){
    arma::vec times;arma::vec floors;arma::vec directions;
    times << 2.42 << 3.21 << 3.56 << 5.67 << 7.01 << 7.05 << 8.90 << 10.10 << 12.12 << 13.53 << 15.14 << arma::endr;
    floors << TERMINAL << TERMINAL << 3 << 4 << TERMINAL << TOP << TOP << 2 << TERMINAL << 5 << TERMINAL << arma::endr;
    directions << UP << UP << UP << DOWN << UP << DOWN << DOWN << UP << UP << DOWN << UP << arma::endr;


    for (int i = 0; i < times.size(); i++){
        Call call(times(i),floors(i),directions(i));
        requests.push(call);
    }

    //Publishers
    //1000 is the message buffer
    pub_elevator_call = nh.advertise<simulation::calls>("/publishCallMsg",1000);
    pub_clock = nh.advertise<std_msgs::Float32>("/clock",1000);

    tstart = std::time(0);
}


void CallGenerator::time_to_publish() {
    simulation::calls msg;
    msg.newCall = false;
    if (requests.size() > 0) {
        Call latest_call = requests.front();

        //if new call has arrived
        if (current_time > latest_call.time) {

            msg.newCall = true;
            msg.time = latest_call.time;
            msg.floor = latest_call.floor;
            msg.direction = latest_call.direction;

            //remove caller from queue
            requests.pop();

            //write to screen
            std::cout << "New caller: " << std::endl;
            std::cout << "new: " << msg.newCall << std::endl;
            std::cout << "time of call: " << msg.time << std::endl;
            std::cout << "from floor: " << msg.floor << std::endl;
            std::cout << "travel direction: " << msg.direction << std::endl;
        }
    }

    //publish
    pub_elevator_call.publish(msg);
}




// ******************************* ROS specific*******************************//

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
        //std::cout << clock << std::endl;

        this->time_to_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {

    /*Initializing ROS system*/
    ros::init(argc, argv, "elevator_call_generator");
    ros::NodeHandle nh;

    CallGenerator *call;
    CallGenerator temp_call(nh);
    call = &temp_call;
    call->spin();
    return 0;
}
