#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "bot_movit/srv/set_joint_p.hpp"

class MoveRobotService : public rclcpp::Node
{
public:
    MoveRobotService() : Node("move_robot_service")
    {
        service_ = this->create_service<bot_movit::srv::SetJointP>(
            "move_robot",
            std::bind(&MoveRobotService::move_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "MoveIt 2 joint control service is ready.");
    }

private:
    rclcpp::Service<bot_movit::srv::SetJointP>::SharedPtr service_;

    void move_callback(const std::shared_ptr<bot_movit::srv::SetJointP::Request> request,
                      std::shared_ptr<bot_movit::srv::SetJointP::Response> response)
    {
        moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "arm");

        // Get available joints dynamically
        const std::vector<std::string> joint_names = move_group.getJointNames();
        if (joint_names.empty())
        {
            response->success = false;
            response->message = "Error: No joints found.";
            return;
        }

        // Fill target positions from request dynamically
        std::map<std::string, double> target_positions;
        if (request->joint_positions.size() != joint_names.size())
        {
            response->success = false;
            response->message = "Error: Provided joint positions do not match the robot's joint count.";
            return;
        }

        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            target_positions[joint_names[i]] = request->joint_positions[i];
        }

        move_group.setJointValueTarget(target_positions);

        // Plan motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            auto exec_result = move_group.execute(plan);
            if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                response->success = true;
                response->message = "Robot moved successfully!";
            }
            else
            {
                response->success = false;
                response->message = "Execution failed.";
            }
        }
        else
        {
            response->success = false;
            response->message = "Planning failed.";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}











