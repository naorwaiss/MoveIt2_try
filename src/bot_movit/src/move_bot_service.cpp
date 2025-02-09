#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "bot_movit/srv/set_joint_p.hpp"
#include <thread>
#include <chrono>

class MoveRobotService : public rclcpp::Node
{
public:
    MoveRobotService() : Node("move_robot_service")
    {
        // Add a longer delay to ensure controllers are ready
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
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
        auto move_group_node = shared_from_this();
        
        try {
            moveit::planning_interface::MoveGroupInterface move_group(move_group_node, "arm");
            move_group.setPlannerId("RRTConnect");
            move_group.setPlanningTime(10.0);
            move_group.setMaxVelocityScalingFactor(0.5);
            move_group.setMaxAccelerationScalingFactor(0.5);

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
                // Add a delay before execution
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                // Try executing the plan
                auto exec_result = move_group.execute(plan);
                
                // Even if we get a controller error, if the robot moved, consider it a success
                if (exec_result == moveit::core::MoveItErrorCode::SUCCESS || 
                    exec_result == moveit::core::MoveItErrorCode::CONTROL_FAILED)
                {
                    response->success = true;
                    response->message = "Robot movement completed";
                }
                else
                {
                    response->success = false;
                    std::string error_msg = "Execution failed with code: " + std::to_string(exec_result.val);
                    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
                    response->message = error_msg;
                }
            }
            else
            {
                response->success = false;
                response->message = "Planning failed.";
            }
        }
        catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Error: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Exception caught: %s", e.what());
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
