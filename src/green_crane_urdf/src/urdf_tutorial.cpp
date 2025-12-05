#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <unistd.h>
#include "AdsLib.h"
#include "AdsVariable.h"
#include "standalone/AdsDef.h"


using namespace std::chrono;

struct AdsVariables
{
	AdsVariables() = delete;

	explicit AdsVariables(AdsDevice& route)
	: boomAng{route, "MAIN.fBoomAng"}
	, rodPressure{route, "MAIN.stAnalogInputs.fP_rod"}
	, pistPressure{route, "MAIN.stAnalogInputs.fP_piston"}
	{
		// Do nothing.
	}

	AdsVariable<double> boomAng; // [rad] angle of crane boom
	AdsVariable<double> rodPressure; // [bar] rod side pressuse
	AdsVariable<double> pistPressure; // [bar] piston side pressuse
};

class AdsComm{
public:
	explicit AdsComm(const AmsNetId remoteNetId, const std::string remoteIpV4)
	:remoteNetId_(remoteNetId)
	,remoteIpV4_(remoteIpV4)
	,route_(remoteIpV4_,remoteNetId_,AMSPORT_R0_PLC_TC3)
	,ads_(route_)
	{}
	
	double getCraneAngle(){
		return ads_.boomAng;
	}
	double getRodPressure(){
		return ads_.rodPressure;
	}
	double getPistPressure(){
		return ads_.pistPressure;
	}
	void printState(){
		const auto state = route_.GetState();
		std::cout << "ADS state: "
							<< std::dec << static_cast<uint16_t>(state.ads)
							<< " devState: "
							<< std::dec << static_cast<uint16_t>(state.device)<<std::endl;
	}
private:
	const AmsNetId remoteNetId_;
	const std::string remoteIpV4_;
	AdsDevice route_;
	AdsVariables ads_;
};

class StatePublisher : public rclcpp::Node{
    public:

    StatePublisher(AdsComm& adsComm,rclcpp::NodeOptions options=rclcpp::NodeOptions())
        :Node("state_publisher",options)
	,adsComm_(adsComm)
	{
        }

        void publish();
	void printState(){
		adsComm_.printState();
	}
	void init(){
            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
            // create a publisher to tell robot_state_publisher the JointState information.
            // robot_state_publisher will deal with this transformation
            //broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            // create a broadcaster to tell the tf2 state information
            // this broadcaster will determine the position of coordinate system 'axis' in coordinate system 'odom'
            RCLCPP_INFO(this->get_logger(),"Starting state publisher");

            //loop_rate_=std::make_shared<rclcpp::Rate>(33ms);

            timer_=this->create_wall_timer(33ms,std::bind(&StatePublisher::publish,this));
	}
    private:
    AdsComm& adsComm_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    //rclcpp::Rate::SharedPtr loop_rate_;
    rclcpp::TimerBase::SharedPtr timer_;
};

void StatePublisher::publish(){
    // create the necessary messages
    sensor_msgs::msg::JointState joint_state;

    // add time stamp
    joint_state.header.stamp=this->get_clock()->now();
    double angle = M_PI/2-adsComm_.getCraneAngle();
    // Specify joints' name which are defined in the r2d2.urdf.xml and their content
    joint_state.name={"base_to_crane_boom"};
    joint_state.position.push_back(angle);


    // send message
    joint_pub_->publish(joint_state);
    std::cout<<"angle: "<<angle<<" [rad] Rod pressure:"<<adsComm_.getRodPressure()<<" [bar], Piston pressure:"<<adsComm_.getPistPressure()<<" [bar]"<<std::endl;

    //RCLCPP_INFO(this->get_logger(),"Publishing joint state");
    //std::cout<<"\033[1A";
}



int main(int argc, char * argv[]){
	rclcpp::init(argc,argv);
	const AmsNetId remoteNetId{192,168,0,10,1,1};
	const std::string remoteIpV4 = "192.168.0.10";
	AdsComm adsComm(remoteNetId,remoteIpV4);
	/*
	std::cout<<"piston Pressure: "<<adsComm.getPistPressure()<<std::endl;
	return 0;
	*/
	auto publisher = std::make_shared<StatePublisher>(adsComm);
	publisher->printState();
	publisher->init();
	rclcpp::spin(publisher);
	rclcpp::shutdown();
	return 0;
}
