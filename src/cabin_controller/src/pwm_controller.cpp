#include "rclcpp/rclcpp.hpp"
#include "cabin_interface/msg/thruster.hpp"
#include "cabin_interface/msg/pwm.hpp"
#include <cmath>

#define PWM_NEUTRAL     1500    // 中立不转
#define PWM_MAX         1800    // 正转最大值
#define PWM_MIN         1200    // 反转最大值

#define PWM_DEADZONE    50      // PWM 死区宽度 ±50

// 推力范围
#define THRUST_MAX      17.0f
#define THRUST_MIN     -17.0f

using namespace std::placeholders;

class ThrusterToPwmNode : public rclcpp::Node
{
public:
    ThrusterToPwmNode() : Node("pwm_controller")
    {
        // 订阅推力话题
        subscriber_ = this->create_subscription<cabin_interface::msg::Thruster>(
            "thruster", 10, std::bind(&ThrusterToPwmNode::callback, this, _1));

        // 发布PWM话题
        publisher_ = this->create_publisher<cabin_interface::msg::Pwm>("pwm", 10);

        RCLCPP_INFO(this->get_logger(), "PWM死区推力转换节点已启动");
    }

private:
    rclcpp::Subscription<cabin_interface::msg::Thruster>::SharedPtr subscriber_;
    rclcpp::Publisher<cabin_interface::msg::Pwm>::SharedPtr publisher_;

    int16_t thrust_to_pwm(float thrust)
    {
        float pwm_val;

        if (thrust > 0) {
            float rate = thrust / THRUST_MAX;
            pwm_val = PWM_NEUTRAL + rate * (PWM_MAX - PWM_NEUTRAL);
        } else {
            float rate = thrust / THRUST_MIN;
            pwm_val = PWM_NEUTRAL - rate * (PWM_NEUTRAL - PWM_MIN);
        }

        if (fabs(pwm_val - PWM_NEUTRAL) <= PWM_DEADZONE) {
            return PWM_NEUTRAL;
        }

        if (pwm_val > PWM_MAX) pwm_val = PWM_MAX;
        if (pwm_val < PWM_MIN) pwm_val = PWM_MIN;

        return static_cast<int16_t>(round(pwm_val));
    }

    void callback(const cabin_interface::msg::Thruster::SharedPtr msg)
    {
        cabin_interface::msg::Pwm pwm_msg;
        pwm_msg.header = msg->header;

        pwm_msg.left_front  = thrust_to_pwm(msg->left_front);
        pwm_msg.right_front = thrust_to_pwm(msg->right_front);
        pwm_msg.left_back   = thrust_to_pwm(msg->left_back);
        pwm_msg.right_back  = thrust_to_pwm(msg->right_back);
        pwm_msg.left_up     = thrust_to_pwm(msg->left_up);
        pwm_msg.right_up    = thrust_to_pwm(msg->right_up);

        publisher_->publish(pwm_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterToPwmNode>());
    rclcpp::shutdown();
    return 0;
}