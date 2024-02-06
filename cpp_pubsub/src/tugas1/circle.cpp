#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	double sudut_rad=0;
	int isFly=0,isDiam=0;
	rclcpp::Time last_publish_time_ = this->get_clock()->now();
	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		offboard_setpoint_counter_ = 0;
		is_land = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				this->arm();
			}

			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			if(is_land == 1) {
                this->land();
                rclcpp::sleep_for(seconds(5));
                this->disarm();
            }

			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();
	void land();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;  

	uint64_t offboard_setpoint_counter_;   
	uint64_t is_land;   

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0);
    RCLCPP_INFO(this->get_logger(), "Land command send");
}

void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};

    double radius = 1.5; 
	if(isFly==0){
		msg.position={0.0,0.0,-5.0};
		msg.yaw=1.57;
		if((this->get_clock()->now() - last_publish_time_).seconds() >= 10.0){
			isFly=1;
			last_publish_time_=this->get_clock()->now();
		}
	}else{
		double x = radius * cos(-sudut_rad);
		double y = radius * sin(-sudut_rad);
		long long sudut=round((sudut_rad)*(180.0/3.141592653589793238463));

		if(sudut==360){
			is_land=1;
		}

		if((sudut==72 || sudut==144 || sudut==216 || sudut==288) && isDiam==0){
			isDiam=1;
			last_publish_time_=this->get_clock()->now();
			RCLCPP_INFO(this->get_logger(), "Stopping..");
		}
		if(isDiam==0){
			sudut_rad+=0.015;
		}else{
			if((this->get_clock()->now() - last_publish_time_).seconds() >= 5.0){
				isDiam=0;
				sudut_rad+=0.015;
			}
		}
		// RCLCPP_INFO(this->get_logger(), "sudut: %lld", sudut%360);
		msg.position = {x, y, -5.0};  
		msg.yaw = -(sudut_rad)-1.57;               
	}

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
    
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}