#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <test_driver/Combined.h>
#include <sensor_msgs/JointState.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <math.h> 
#include <stdlib.h> 
#include <random>
using namespace std;

#define PI 3.14159265
#define link_1 0.5
#define link_2 1.0
ros::Publisher* first_pub;
ros::Publisher* second_pub;
mt19937 rng;

struct Pose {
    double x;
    double y;
    double theta;
};
long exp_count;
vector< vector<Pose>  > tracker;
/*void Callback(const ... & msg) {
    
}
*/





// Reset sin variables
// A value is amplitude and it is about how fast the joint
// B value is frequency and calculates the frequency of the action
// delay is in order to be able set some time apart between actions
void resetSinVariables(double &first_A, double &first_B, double &first_delay, double &second_A,  double &second_B, double &second_delay) {

    #include <random>

    uniform_int_distribution<int> first_A_ran(-10,10); // guaranteed unbiased
    uniform_int_distribution<int> first_B_ran(1,4); // guaranteed unbiased
    uniform_int_distribution<int> first_delay_ran(1,150);
    uniform_int_distribution<int> second_A_ran(-10,10); // guaranteed unbiased
    uniform_int_distribution<int> second_B_ran(1,4); // guaranteed unbiased
    uniform_int_distribution<int> second_delay_ran(1,150);



    first_A =first_A_ran(rng)/10.0;
    if(first_A == 0) {          /// velocity should not be zero
        first_A = 1;
    }
    first_B = first_B_ran(rng) ;
    first_delay = first_delay_ran(rng) /100.0;


    second_A = second_A_ran(rng)/10.0;
    if(second_A == 0) {
        second_A = 1;
    }
    second_B = second_B_ran(rng);
    second_delay = second_delay_ran(rng)/100.0;
    
    cout<<"--------------------------------------------------------------------------------------"<<endl;
    cout<<"First joint sin values A: "<<first_A<<", B: "<<first_B<<" and delay: "<<first_delay<<endl;
    cout<<"Second joint sin values A: "<<second_A<<", B: "<<second_B<<" and delay: "<<second_delay<<endl;
    cout<<"--------------------------------------------------------------------------------------"<<endl;
}

// Reset timers in order to be able do multiple tests
void resetTimers(int publish_rate ,int &action_time, int &first_count_until, int &second_count_until, int &first_counter, int &second_counter) {
    
    first_counter = 0;
    second_counter = 0;
    uniform_int_distribution<int> action_time_ran(3,6);
    action_time = action_time_ran(rng);      // how much time it will take to finish the action
    
    first_count_until = publish_rate*action_time;             // need to connect with frequency
    second_count_until = publish_rate*action_time;
    cout<<"--------------------------------------------------------------------------------------"<<endl;
    cout<<"New Timer values after reset"<<endl;
    cout<<"How long will action take: "<<action_time<<", first joint count until: "<<first_count_until<<", second joint count until: "<<second_count_until<<endl;
    cout<<"--------------------------------------------------------------------------------------"<<endl;
}

void jointPositionTracker(const sensor_msgs::JointState & in_msg) {
    Pose link_1_pose;
    Pose link_2_pose;
    link_1_pose.theta = in_msg.position[0];
    link_1_pose.y = link_1*cos(in_msg.position[0]);     //reversed the x and y because of gazebo's axises
    link_1_pose.x = link_1*sin(in_msg.position[0]);
    link_2_pose.theta = in_msg.position[1];
    link_2_pose.y = link_1_pose.x+ link_2*cos(in_msg.position[0]+in_msg.position[1]);
    link_2_pose.x = link_1_pose.y+ link_2*sin(in_msg.position[0]+in_msg.position[1]);
    vector<Pose> temp_vector;
    temp_vector.push_back(link_1_pose);
    temp_vector.push_back(link_2_pose);
    tracker.push_back(temp_vector);
}


void recordAction(double first_A, double first_B, double first_delay, double first_count_until, double second_A, double second_B, double second_delay, double second_count_until) {
    cout<<exp_count<<endl;
    boost::filesystem::path p{"/home/fatih/collected/"+to_string(exp_count)+"/data.txt"};
    boost::filesystem::ofstream ofs{p};
    ofs<<"link1_length: "<<link_1<<",link2_length: "<<link_2 <<endl;
    ofs<<"f_A \t f_B \t f_D \t f_C_U \t s_A \t s_B \t s_D \t s_C_U"<<endl;
    ofs<<first_A<<" \t "<<first_B<<" \t "<<first_delay<<" \t "<<first_count_until<<" \t "<<second_A<<" \t "<<second_B<<" \t "<<second_delay<<" \t "<<second_count_until<<endl;
    ofs<<"t_1 \t x_1 \t y_1 \t t_2 \t x_2 \t y_2"<<endl; 
    for(int i=0;i<tracker.size();i++) {
        ofs<<tracker[i][0].theta<<" "<< tracker[i][0].x<<" "<<tracker[i][0].y <<" "<<tracker[i][1].theta<<" "<<tracker[i][1].x<<" "<<tracker[i][1].y<<endl;
    }
}

int main(int argc,char** argv) {

    ros::init(argc, argv, "velocity_driver");
    ros::NodeHandle nh;
     srand (time(NULL));
    //ros::Subscriber sub = nh.subscribe("TopicToFollow",Rate,CallbackFunctionName);


    random_device rd;     // only used once to initialise (seed) engine
    rng.seed(rd());    // random-number engine used (Mersenne-Twister in this case)    
    uniform_int_distribution<int> first_A_ran(-10,10); // guaranteed unbiased
    uniform_int_distribution<int> first_B_ran(1,4); // guaranteed unbiased
    uniform_int_distribution<int> first_delay_ran(1,150);
    uniform_int_distribution<int> second_A_ran(-10,10); // guaranteed unbiased
    uniform_int_distribution<int> second_B_ran(1,4); // guaranteed unbiased
    uniform_int_distribution<int> second_delay_ran(1,150);
    uniform_int_distribution<int> action_time_ran(3,6);
    double param ;
    double turned;// =  param*PI/180;
    double first_sin_value = 0.0;
    double first_A = first_A_ran(rng)/10.0; //((rand() % 100 + 1)-50.0)/10.0;
    if(first_A == 0) {
        first_A = 1;
    }
    double first_B = first_B_ran(rng) ; //(rand() % 4 + 1);
    double first_C = 0;//rand() % 10 + 1;;
    double first_D = 0;//(rand() % 10 + 1)/10;
    double first_delay =  first_delay_ran(rng) /100.0; //((rand() % 150 + 1))/100.0; 

    double second_sin_value = 0.0;
    double second_A =  second_A_ran(rng)/10.0;  //((rand() % 100 + 1)-50.0)/10.0;
    if(second_A == 0) {
        second_A = 1;
    }
    double second_B =  second_B_ran(rng);  //(rand() % 4 + 1);
    double second_C = 0;//rand() % 10 + 1;;
    double second_D = 0;//(rand() % 10 + 1)/10;
    double second_delay =  second_delay_ran(rng)/100.0; //((rand() % 150 + 1))/100.0;

    int first_counter = 0;
    int second_counter = 0;

    int action_time = action_time_ran(rng);

    int first_count_until = 0;
    int second_count_until = 0;
    
    int free_wait_count = 0;
    exp_count = 0;

    ROS_INFO_STREAM("Velocity Driver is On");
    
    first_pub = new ros::Publisher(nh.advertise<std_msgs::Float64>("/test_robot/first_joint_controller/command",1));
    second_pub = new ros::Publisher(nh.advertise<std_msgs::Float64>("/test_robot/second_joint_controller/command",1));
    ros::Publisher combined_pub = nh.advertise<test_driver::Combined>("/test_robot/combined_commands",1);
    ros::Subscriber sub = nh.subscribe("/test_robot/joint_states",1,&jointPositionTracker);

    // setting the recording place

    

    int publish_rate = 5;

    ros::Rate rate(publish_rate);
    bool first_start = false;
    bool second_start = false;
    bool first_finished = true;
    bool second_finished = true;


    while(ros::ok()) {



        ros::spinOnce();

        // First joint calculations
        if( (first_start == false) && (first_counter> first_delay*publish_rate) ){ /// apply the delay and start the actual counting process
            first_start = true;
            first_counter = 0;

        } else if( (first_start == false) && (first_counter <= first_delay*publish_rate) ) {
            first_counter++;
        }
        if( (first_start == true) && (first_finished == false) )  {
            // can use first_A value directly
            // should do some tweek to be able to use first_B(frequency)
            // should traverse pi/B in order to complete the upper part of the sin function and end up in 0 value
            // but should also change first_count_until value too
            // not sure how to combine them 
            // if done correctly it will be end of the data part for the time being

            param = (PI/(action_time*publish_rate))*first_counter;
            if(param> PI/first_B){
                first_finished = true;
                first_sin_value = 0.0; // zero if action is completed;
            }
            else {

                first_sin_value = first_A*(sin(first_B*param));
                first_counter++;    
            }
            


        }
        if( (first_finished == false) && (first_counter > first_count_until) ) {
            first_finished =true;
            first_sin_value = 0.0;
        } 
        if( first_finished == true ) {
            first_sin_value = 0.0;
        }


        // Second joint calculations
        if( (second_start == false) && (second_counter> second_delay*publish_rate)  ){ /// apply the delay and start the actual counting process
            second_start = true;
            second_counter = 0;
        } else if( (second_start == false) && (second_counter <= second_delay*publish_rate) ) {
            second_counter++;
        }
        if( (second_start == true) && (second_finished == false) )  {
            // can use second_A value directly
            // should do some tweek to be able to use second_B(frequency)
            // should traverse pi/B in order to complete the upper part of the sin function and end up in 0 value
            // but should also change second_count_until value too
            // not sure how to combine them 
            // if done correctly it will be end of the data part for the time being

            param = (PI/(action_time*publish_rate))*second_counter;
            if(param> PI/second_B){
                second_finished = true;
                second_sin_value = 0.0; // zero if action is completed;
            }
            else {
                
                second_sin_value = second_A*(sin(second_B*param));
                second_counter++;    
            }
            


        }
        if( (second_counter > second_count_until) && (second_finished == false) ) {
            second_finished =true;
            second_sin_value = 0.0;
        } 
        if( second_finished == true ) {
            second_sin_value = 0.0;
        }

        if(first_finished == true && second_finished == true && (free_wait_count ==0)){
            free_wait_count++;
           
            boost::filesystem::path p{"/home/fatih/collected/"+to_string(exp_count)};
            try
            {
                if(!boost::filesystem::exists(p)) {
                    boost::filesystem::create_directory(p);
                }
                
            }
            catch (boost::filesystem::filesystem_error &e)
            {
                std::cerr << e.what() << '\n';
            }
            recordAction(first_A, first_B, first_delay, first_count_until, second_A, second_B, second_delay, second_count_until);
            exp_count++;
        }
        if(first_finished == true && second_finished == true && (free_wait_count <5)){
            free_wait_count++;
        }
        else if(first_finished == true && second_finished == true && (free_wait_count >= 5) ) { /// resetting everything and starting a new test
            resetSinVariables(first_A, first_B, first_delay, second_A,  second_B, second_delay);
            resetTimers(publish_rate ,action_time, first_count_until, second_count_until, first_counter, second_counter);
            free_wait_count = 0;
            first_start = false;
            second_start = false;

            first_finished = false;
            second_finished = false;
            tracker.clear();
        }

        std_msgs::Float64 joint1_msg;
        std_msgs::Float64 joint2_msg;

        test_driver::Combined com_msg;



        /*ROS_INFO_STREAM("just sin ="<< sin(param) );
        ROS_INFO_STREAM("sin with freq change ="<<sin(first_B*param));
        ROS_INFO_STREAM(joint1_msg.data);
        */
        joint1_msg.data=first_sin_value;
        joint2_msg.data=second_sin_value; 
        com_msg.first_joint = first_sin_value;
        com_msg.second_joint = second_sin_value;

        first_pub->publish(joint1_msg);
        second_pub->publish(joint2_msg);

        combined_pub.publish(com_msg);
        rate.sleep();
    }
    


    delete first_pub;
    delete second_pub;
}
