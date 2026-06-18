#include "cabin_teleop/joy_controller.hpp"

JoyController:: JoyController()
 : rclcpp::Node("joystick_sub", "cabin_auv"), count_(0){
    using std::placeholders::_1;
    // joystick param init
    joystick_param_init();
    //subscribe the joystick signal and update the param
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
         "joy", rclcpp::SensorDataQoS(), std::bind(&JoyController::JoyCallback, this, _1));

    //load the current param and pub NetLoad msg
    joy_move_pub = this->create_publisher<cabin_interface::msg::ControlMove>("command/move", 1);
    timer = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&JoyController::timer_callback, this));

    //the lock for safety
    joy_cmd_pub = this->create_publisher<cabin_interface::msg::ControlCmd>("command/cmd", 1);    
 }

void JoyController::joystick_param_init(){
        joy_switch_state = 0;
        pid_enable = false;
        max_forward_thrust = 33.0;
        max_backward_thrust = 33.0;
        max_forward_thrust_ud = 33.0;
        max_backward_thrust_ud = 33.0;
}

void JoyController::JoyCallback(const sensor_msgs::msg::Joy msg){
    joy_force[2] = 0;
    if(1 == msg.axes[AXES_CROSS_UD]){
        joy_force[2] = 15;
        RCLCPP_INFO(get_logger(), "robot up!!!");
    }
    else if(-1 == msg.axes[AXES_CROSS_UD]){
        joy_force[2] = -15;
        RCLCPP_INFO(get_logger(), "robot down!!!");
    }

    if(msg.axes[AXES_STICK_LEFT_UD]){
        current_axes_factor[0] = msg.axes[AXES_STICK_LEFT_UD];
        if(current_axes_factor[0] > 0){
            joy_force[0] = current_axes_factor[0] * max_forward_thrust;
        }
        else{
            joy_force[0] = current_axes_factor[0] * max_backward_thrust;
        }
    }

    if(msg.axes[AXES_STICK_LEFT_LR]){
        current_axes_factor[1] = msg.axes[AXES_STICK_LEFT_LR];
        if(current_axes_factor[1] > 0){
            joy_force[1] = -current_axes_factor[1] * max_forward_thrust;
        }
        else{
            joy_force[1] = -current_axes_factor[1] * max_backward_thrust;
        }
    }

    if(msg.axes[AXES_STICK_RIGHT_LR]){
        current_axes_factor[2] = msg.axes[AXES_STICK_RIGHT_LR];
        if(current_axes_factor[2] < 0){
            joy_moment[2] = current_axes_factor[2] * max_forward_thrust_ud;
        }
        else{
            joy_moment[2] = current_axes_factor[2] * max_backward_thrust_ud;
        }
    }

    // the lock button for safety
    if(1 == msg.buttons[BUTTON_SHAPE_A]){
        joy_switch_state = (joy_switch_state+1)%3;
        rclcpp::Time now = this->now();
        cmd.header.stamp = now;
        cmd.lock = joy_switch_state;
        joy_cmd_pub->publish(cmd);
        if(joy_switch_state == 0){
            RCLCPP_INFO(get_logger(), "control locked!!!");
        }
        else if(joy_switch_state == 1){
            RCLCPP_INFO(get_logger(), "joy control!!!");
        }else{
            RCLCPP_INFO(get_logger(), "computer control!!!");
        }
    }
    // reset the input
    if( 1 == msg.buttons[BUTTON_SHAPE_B]){
        for(int i = 0; i < 3; i++){
            joy_force[i] = 0.0;
            joy_moment[i] = 0.0;
        }
        RCLCPP_INFO(get_logger(), "joystick input has been reset!!!");
    }
    if( 1 == msg.buttons[BUTTON_SHAPE_X]){
        cmd.imu_reset = 1;
        rclcpp::Time now = this->now();
        cmd.header.stamp = now;
        joy_cmd_pub->publish(cmd);
        cmd.imu_reset = 0;
    }
    if(1 == msg.buttons[BUTTON_SHAPE_Y]){
        pid_enable = !pid_enable;
        rclcpp::Time now = this->now();
        cmd.header.stamp = now;
        cmd.pid_enable = pid_enable;
        joy_cmd_pub->publish(cmd);
        if(pid_enable){
            RCLCPP_INFO(get_logger(), "pid start!!!");
        }
        else{
            RCLCPP_INFO(get_logger(), "pid end!!!");
        }
    }
}


void JoyController::timer_callback(){
    //current timestamp
    rclcpp::Time now = this->now();
    move.header.stamp = now;
    if(joy_moment[2]>-2 && joy_moment[2]<2){
        joy_moment[2] = 0;
    }
    //Move forward
    move.force.x = joy_force[0];
    move.force.y = joy_force[1];
    move.force.z = joy_force[2];
    //Rotate around z axis (yaw)
    move.moment.x = joy_moment[0];
    move.moment.y = joy_moment[1];
    move.moment.z = joy_moment[2]; 

    joy_move_pub->publish(move);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor joy_sub_executor;
    auto joy_sub_node = std::make_shared<JoyController>();
    joy_sub_executor.add_node(joy_sub_node);
    joy_sub_executor.spin();
    joy_sub_executor.remove_node(joy_sub_node);
    return 0;
}
