#include <rclcpp/rclcpp.hpp>
#include "crane_interface/msg/crane_ctrl.hpp"
#include "AdsLib.h"
#include "AdsVariable.h"
#include "standalone/AdsDef.h"

using namespace std::chrono;

struct AdsVariables
{
	AdsVariables() = delete;

	explicit AdsVariables(AdsDevice& route)
	: angleRef{route, "MAIN.fBoomAngRef"}
	, joystick{route, "MAIN.stAnalogInputs.fJoystick"}
	, startStop{route, "MAIN.stButtonInputs.bEnableCraneMotion"}
	{
		// Do nothing.
	}

	AdsVariable<double> angleRef; // [rad] angle of crane boom reference
	AdsVariable<double> joystick; // joystick input
	AdsVariable<bool> startStop; // [bar] rod side pressuse
};

class AdsComm{
public:
	explicit AdsComm(const AmsNetId remoteNetId, const std::string remoteIpV4)
	:remoteNetId_(remoteNetId)
	,remoteIpV4_(remoteIpV4)
	,route_(remoteIpV4_,remoteNetId_,AMSPORT_R0_PLC_TC3)
	,ads_(route_)
	{}
	
	void setStartStop(bool val){
		ads_.startStop = val;
	}
	void setJoystick(double val){
		ads_.joystick = val;
	}
	void setAngle(double val){
		ads_.angleRef = val;
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


class CraneSubscriber : public rclcpp::Node
{
public:
    CraneSubscriber(AdsComm& adsComm)
    : Node("crane_subscriber")
    , adsComm_(adsComm)
    {
        // Create a subscriber with the custom message type
        subscription_ = this->create_subscription<crane_interface::msg::CraneCtrl>(
            "crane_topic", 10,
            std::bind(&CraneSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    // Callback function executed every time a new message is received
    void topic_callback(const crane_interface::msg::CraneCtrl::SharedPtr msg) const {
	adsComm_.setStartStop(msg->start_stop);
	adsComm_.setJoystick(msg->joystick);
	adsComm_.setAngle(msg->boom_angle);
        RCLCPP_INFO(this->get_logger(), "Received Crane Control Data: Boom Angle = %.2f, Start/Stop = %s",
                    msg->boom_angle, msg->start_stop ? "True" : "False");
    }

    AdsComm& adsComm_;
    rclcpp::Subscription<crane_interface::msg::CraneCtrl>::SharedPtr subscription_;
};

int main(int argc, char *argv[]){
	rclcpp::init(argc,argv);
	const AmsNetId remoteNetId{192,168,0,10,1,1};
	const std::string remoteIpV4 = "192.168.0.10";
	AdsComm adsComm(remoteNetId,remoteIpV4);

	rclcpp::spin(std::make_shared<CraneSubscriber>(adsComm));
	rclcpp::shutdown();
	return 0;
}

