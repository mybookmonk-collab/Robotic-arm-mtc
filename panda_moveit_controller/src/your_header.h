#ifndef YOUR_PACKAGE_YOUR_HEADER_H
#define YOUR_PACKAGE_YOUR_HEADER_H

#include <moveit/move_group_interface/move_group_interface.h>
// INCLUDE YOUR SRVS 


class MoveRobot : public rclcpp::Node {
public:
    explicit MoveRobot(const rclcpp::NodeOptions &options);

private:
    rclcpp::Service<your_service_msgs::srv::ExampleService>::SharedPtr service_example_server_;
    std::string node_namespace_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;

    void service_example_callback(const std::shared_ptr<your_service_msgs::srv::ExampleService::Request> request,
                                  std::shared_ptr<your_service_msgs::srv::ExampleService::Response> response);
};

#endif //YOUR_PACKAGE_YOUR_HEADER_H
