#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cabin_interface/msg/control_cmd.hpp>
#include <cmath>

struct RotMat3
{
    double m[3][3];
    RotMat3()
    {
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) m[i][j] = 0.0;
    }
};

geometry_msgs::msg::Vector3 vec3_mult_mat(const geometry_msgs::msg::Vector3& v, const RotMat3& mat)
{
    geometry_msgs::msg::Vector3 res;
    res.x = mat.m[0][0] * v.x + mat.m[0][1] * v.y + mat.m[0][2] * v.z;
    res.y = mat.m[1][0] * v.x + mat.m[1][1] * v.y + mat.m[1][2] * v.z;
    res.z = mat.m[2][0] * v.x + mat.m[2][1] * v.y + mat.m[2][2] * v.z;
    return res;
}

RotMat3 get_inv_rpy_mat(double r, double p, double y)
{
    RotMat3 mat;
    double cr = cos(r), sr = sin(r);
    double cp = cos(p), sp = sin(p);
    double cy = cos(y), sy = sin(y);

    mat.m[0][0] = cy*cp;
    mat.m[0][1] = cy*sp*sr - sy*cr;
    mat.m[0][2] = cy*sp*cr + sy*sr;

    mat.m[1][0] = sy*cp;
    mat.m[1][1] = sy*sp*sr + cy*cr;
    mat.m[1][2] = sy*sp*cr - cy*sr;

    mat.m[2][0] = -sp;
    mat.m[2][1] = cp*sr;
    mat.m[2][2] = cp*cr;
    return mat;
}

class ImuTransformNode : public rclcpp::Node
{
public:
    ImuTransformNode(const rclcpp::NodeOptions & options)
    : Node("imu_transform_node", options)
    {
        this->declare_parameter<std::string>("pub_imu_topic", "tf/imu");
        this->declare_parameter<std::string>("pub_euler_topic", "tf/euler_angles");
        this->declare_parameter<std::string>("imu_output_frame", "imu_base");

        std::string imu_topic = this->get_parameter("pub_imu_topic").as_string();
        std::string euler_topic = this->get_parameter("pub_euler_topic").as_string();
        output_frame_id_ = this->get_parameter("imu_output_frame").as_string();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&ImuTransformNode::imu_cb, this, std::placeholders::_1));
        euler_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "euler_angles", 10, std::bind(&ImuTransformNode::euler_cb, this, std::placeholders::_1));
        cmd_sub_ = this->create_subscription<cabin_interface::msg::ControlCmd>(
            "command/cmd", 10, std::bind(&ImuTransformNode::cmd_cb, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
        euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(euler_topic, 10);

        transform_valid_ = false;
        base_rad_.x = base_rad_.y = base_rad_.z = 0.0;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr euler_sub_;
    rclcpp::Subscription<cabin_interface::msg::ControlCmd>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub_;

    sensor_msgs::msg::Imu::SharedPtr cache_imu_;
    geometry_msgs::msg::Vector3::SharedPtr cache_euler_;
    bool transform_valid_;
    geometry_msgs::msg::Vector3 base_rad_;
    RotMat3 inv_base_mat_;
    std::string output_frame_id_;

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        cache_imu_ = msg;
        publish_output();
    }

    void euler_cb(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        cache_euler_ = msg;
        publish_output();
    }

    void cmd_cb(const cabin_interface::msg::ControlCmd::SharedPtr msg)
    {
        if(!msg->imu_reset || !cache_euler_) return;
        base_rad_.x = cache_euler_->x * M_PI / 180.0;
        base_rad_.y = cache_euler_->y * M_PI / 180.0;
        base_rad_.z = cache_euler_->z * M_PI / 180.0;
        inv_base_mat_ = get_inv_rpy_mat(base_rad_.x, base_rad_.y, base_rad_.z);
        transform_valid_ = true;
    }

    void publish_output()
    {
        if(!cache_imu_ || !cache_euler_) return;
        if(!transform_valid_)
        {
            imu_pub_->publish(*cache_imu_);
            euler_pub_->publish(*cache_euler_);
            return;
        }

        sensor_msgs::msg::Imu out_imu = *cache_imu_;
        out_imu.header.frame_id = output_frame_id_;
        geometry_msgs::msg::Vector3 out_euler;

        out_imu.linear_acceleration = vec3_mult_mat(cache_imu_->linear_acceleration, inv_base_mat_);
        out_imu.angular_velocity = vec3_mult_mat(cache_imu_->angular_velocity, inv_base_mat_);

        double curr_r = cache_euler_->x * M_PI / 180.0;
        double curr_p = cache_euler_->y * M_PI / 180.0;
        double curr_y = cache_euler_->z * M_PI / 180.0;

        double rel_r = curr_r - base_rad_.x;
        double rel_p = curr_p - base_rad_.y;
        double rel_y = curr_y - base_rad_.z;

        out_euler.x = rel_r * 180.0 / M_PI;
        out_euler.y = rel_p * 180.0 / M_PI;
        out_euler.z = rel_y * 180.0 / M_PI;

        while (out_euler.z < 0.0)
        {
            out_euler.z += 360.0;
        }
        while (out_euler.z >= 360.0)
        {
            out_euler.z -= 360.0;
        }

        imu_pub_->publish(out_imu);
        euler_pub_->publish(out_euler);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<ImuTransformNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}