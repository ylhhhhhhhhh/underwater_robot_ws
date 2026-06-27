#include "rclcpp/rclcpp.hpp"
#include "cabin_interface/msg/control_move.hpp"
#include "cabin_interface/msg/thruster.hpp"
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

class ThrusterAllocator {
public:
    ThrusterAllocator(const MatrixXd& T) : T_(T) {}

    VectorXd solve(const VectorXd& tau_des) {
        double u[6] = {0};

        ceres::Problem problem;
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<Cost, 6, 6>(new Cost(T_, tau_des)),
            nullptr, u);

        for (int i = 0; i < 6; i++) {
            problem.SetParameterLowerBound(u, i, -33.0);
            problem.SetParameterUpperBound(u, i, 33.0);
        }

        ceres::Solver::Options options;
        options.max_num_iterations = 30;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        return Eigen::Map<VectorXd>(u, 6);
    }

private:
    MatrixXd T_;

    struct Cost {
        MatrixXd T_mat_;
        VectorXd tau_des_;

        Cost(const MatrixXd& T, const VectorXd& tau) : T_mat_(T), tau_des_(tau) {}

        template <typename T>
        bool operator()(const T* const u, T* residual) const {
            Eigen::Matrix<T, 6, 1> tau = T_mat_.cast<T>() * Eigen::Map<const Eigen::Matrix<T, 6, 1>>(u);
            Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residual);
            res = tau_des_.cast<T>() - tau; 
            return true;
        }
    };
};

class ThrusterController : public rclcpp::Node {
public:
    ThrusterController() : Node("thruster_controller") {
        cmd_sub_ = this->create_subscription<cabin_interface::msg::ControlMove>(
            "command/pid/move", 10,
            std::bind(&ThrusterController::callback, this, std::placeholders::_1));

        thrust_pub_ = this->create_publisher<cabin_interface::msg::Thruster>("thruster", 10);

        double x = sqrt(2) / 2;
        MatrixXd T(6,6);
        T <<  x,  x,  x,  x,  0,  0,   
              x, -x, -x,  x,  0,  0,   
              0,  0,  0,  0, -1, -1,   
              0,  0,  0,  0,  0,  0,   
              0,  0,  0,  0,  0,  0,   
              1, -1,  1, -1,  0,  0;   

        allocator_ = std::make_shared<ThrusterAllocator>(T);
    }

private:
    void callback(const cabin_interface::msg::ControlMove::SharedPtr msg) {
        VectorXd tau(6);
        tau << msg->force.x, msg->force.y, msg->force.z,
               msg->moment.x, msg->moment.y, msg->moment.z;

        VectorXd u = allocator_->solve(tau);

        cabin_interface::msg::Thruster out;
        out.left_front  = u[0];
        out.right_front = u[1];
        out.left_back   = u[2];
        out.right_back  = u[3];
        out.left_up     = u[4];
        out.right_up    = u[5];

        // RCLCPP_INFO(get_logger(), "U: %.1f %.1f %.1f %.1f %.1f %.1f",
        //             u[0],u[1],u[2],u[3],u[4],u[5]);

        thrust_pub_->publish(out);
    }

    rclcpp::Subscription<cabin_interface::msg::ControlMove>::SharedPtr cmd_sub_;
    rclcpp::Publisher<cabin_interface::msg::Thruster>::SharedPtr thrust_pub_;
    std::shared_ptr<ThrusterAllocator> allocator_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrusterController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
