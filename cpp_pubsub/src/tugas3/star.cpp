#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		offboard_setpoint_counter_ = 0;
		waypoint_index = 0;
        is_land = 0;
		last_publish_time_ = this->get_clock()->now();

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
	uint8_t waypoint_index;
    uint8_t is_land;
	rclcpp::Time last_publish_time_; 

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
	std::vector<std::array<float, 4>> waypoints = {
        // star
        {0.0f, 0.0f, -5.0f, 0.0f},
        {-2.707f, 0.891f, -5.0f, -1.257f},
        {-1.05f, -1.431f, -5.0f, 0.6283f},
        {-1.05f, 1.431f, -5.0f, 0.6283f},
        {-2.707f, -0.891f, -5.0f, 0.6283f},
        {0.0f, 0.0f, -5.0f, 0.6283f},
        {0.0f, 0.0f, 0.0f, 0.0f}
    };
	msg.position = {waypoints[waypoint_index][0], waypoints[waypoint_index][1], waypoints[waypoint_index][2]};
	msg.yaw = waypoints[waypoint_index][3];
	RCLCPP_INFO(this->get_logger(), "Current Target Position: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", msg.position[0], msg.position[1], msg.position[2], msg.yaw);
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	if ((this->get_clock()->now() - last_publish_time_).seconds() >= 10.0) {
        this->waypoint_index++;
		if(this->waypoint_index==7){
			is_land = 1;
		}
        last_publish_time_= this->get_clock()->now();
    }
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