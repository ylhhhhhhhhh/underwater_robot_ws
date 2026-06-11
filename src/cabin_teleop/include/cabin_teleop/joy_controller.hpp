#ifndef JOY_CONTROLLER_HPP_
#define JOY_CONTROLLER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "cabin_teleop/t4_button_mapping.h"

#include "sensor_msgs/msg/joy.hpp"
#include "cabin_interface/msg/control_move.hpp"
#include "cabin_interface/msg/control_cmd.hpp"

class JoyController : public rclcpp::Node{
    public:
        JoyController();
        virtual ~JoyController() = default;

    private:
        // joystick param
        double joy_force[3];                  //The force input along the x, y, z axis
        double joy_moment[3];                 //The moment input around the x, y, axis
        bool joy_switch_state;
        double max_forward_thrust;
        double max_backward_thrust;
        double max_forward_thrust_ud;
        double max_backward_thrust_ud;
        double current_axes_factor[4];

        cabin_interface::msg::ControlMove move;
        cabin_interface::msg::ControlCmd cmd;
        
        void joystick_param_init();        
        // subscribe the joystick signal
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        void JoyCallback(const sensor_msgs::msg::Joy msg);
        // publish the control signal
        rclcpp::Publisher<cabin_interface::msg::ControlMove>::SharedPtr joy_move_pub;
        rclcpp::TimerBase::SharedPtr timer;
        void timer_callback();
        // lock state and reset state
        rclcpp::Publisher<cabin_interface::msg::ControlCmd>::SharedPtr joy_cmd_pub;
        
        size_t count_;
};

#endif