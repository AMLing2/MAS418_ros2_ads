#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <unistd.h>
#include "AdsLib.h"
#include "AdsVariable.h"
#include "standalone/AdsDef.h"

using namespace std::chrono;

class StatePublisher : public rclcpp::Node{
    public:

    StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
        Node("state_publisher",options){
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

        void publish();
    private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    //rclcpp::Rate::SharedPtr loop_rate_;
    rclcpp::TimerBase::SharedPtr timer_;

    //Robot state variables
    // degree means one degree
    const double degree=M_PI/180.0;
    double angle = 0.;
    double cnt = 0.;
};

void StatePublisher::publish(){
    // create the necessary messages
    sensor_msgs::msg::JointState joint_state;

    // add time stamp
    joint_state.header.stamp=this->get_clock()->now();
    // Specify joints' name which are defined in the r2d2.urdf.xml and their content
    joint_state.name={"base_to_crane_boom"};
    joint_state.position.push_back(angle);

    // update state for next time
		cnt+=0.1;
		angle=sin(cnt)*M_PI/10.0;

    // send message
    joint_pub_->publish(joint_state);

    RCLCPP_INFO(this->get_logger(),"Publishing joint state");
}

struct AdsVariables
{
	AdsVariables() = delete;

	explicit AdsVariables(AdsDevice& route)
	: craneAngle{route, "MAIN.bActivateMotion"}
	// , velocityReference{route, "MAIN.fVelRef"}
	// , positionReference{route, "MAIN.fPosRef"}
	// , positionMeasurement{route, "MAIN.fPosMeas"}
	{
		// Do nothing.
	}

	AdsVariable<double> craneAngle;
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
		return ads_.craneAngle;
	}
	void printState(){
		const auto state = route_.GetState();
		std::cout << "ADS state: "
							<< std::dec << static_cast<uint16_t>(state.ads)
							<< " devState: "
							<< std::dec << static_cast<uint16_t>(state.device);
	}
private:
	const AmsNetId remoteNetId_;
	const std::string remoteIpV4_;
	AdsDevice route_;
	AdsVariables ads_;
};


int main(int argc, char * argv[]){
	rclcpp::init(argc,argv);
	const AmsNetId remoteNetId {192,168,0,10,1,1};
	const std::string remoteIpV4 = "192.168.0.10";
	AdsComm adsComm(remoteNetId,remoteIpV4);
	adsComm.printState();
	std::cout<<"angle:"<<adsComm.getCraneAngle()<<std::endl;
	return 0;
	rclcpp::spin(std::make_shared<StatePublisher>());
	rclcpp::shutdown();
	return 0;
}
