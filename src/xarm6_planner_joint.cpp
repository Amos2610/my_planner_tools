#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>

class XArmPlannerNode {
public:
    XArmPlannerNode(const rclcpp::NodeOptions& options)
    {
        node_ = rclcpp::Node::make_shared("xarm_planner_node", options);
        joint_plan_client_ = node_->create_client<xarm_msgs::srv::PlanJoint>("xarm_joint_plan");
        exec_plan_client_ = node_->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");

        // // Declare parameter
        // node_->declare_parameter<std::vector<double>>(
        //     "target_joint", {}, rcl_interfaces::msg::ParameterDescriptor{});

        // パラメータ取得
        node_->get_parameter_or("target_joint", target_joint_, std::vector<double>{});

        // ログ出力
        if (target_joint_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "target_joint is empty. Please specify it via --ros-args -p target_joint:=...");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Received target_joint with %ld elements", target_joint_.size());
        }
        // Get target_joint param
        if (!node_->get_parameter("target_joint", target_joint_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get 'target_joint' parameter");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Received target_joint with %ld elements", target_joint_.size());
        }
    }

    bool plan()
    {
        if (target_joint_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "target_joint is empty. Aborting plan.");
            return false;
        }

        auto req = std::make_shared<xarm_msgs::srv::PlanJoint::Request>();
        req->target = target_joint_;
        return call_service(joint_plan_client_, req);
    }

    bool execute(bool wait = true)
    {
        auto req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
        req->wait = wait;
        return call_service(exec_plan_client_, req);
    }

    rclcpp::Node::SharedPtr get_node() { return node_; }

private:
    rclcpp::Node::SharedPtr node_;
    std::vector<double> target_joint_;
    rclcpp::Client<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_client_;
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_;

    template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
    bool call_service(std::shared_ptr<ServiceT> client, SharedRequest req)
    {
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return false;
            RCLCPP_WARN(node_->get_logger(), "Waiting for service %s...", client->get_service_name());
        }

        auto future = client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call %s", client->get_service_name());
            return false;
        }

        return future.get()->success;
    }
};

void exit_sig_handler(int) {
    fprintf(stderr, "Ctrl-C detected. Shutting down.\n");
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, exit_sig_handler);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);

    auto planner = std::make_shared<XArmPlannerNode>(options);

    if (planner->plan()) {
        planner->execute();
    }

    rclcpp::shutdown();
    return 0;
}
