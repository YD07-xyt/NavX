#include"decision.h"
#include<iostream>
namespace decision {
    FSM::FSM(rclcpp::Node::SharedPtr node):node_(node),last_sent_goal_(0,0,0){
        this->goal_pub_ =
             node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        this->nav2_status_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status",
            10,
            std::bind(&FSM::nav2_status_callback, this, std::placeholders::_1)
        );
    }
    void FSM::nav2_status_callback(const action_msgs::msg::GoalStatusArray msg) {
        for (const auto& status : msg.status_list) {
            if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
                this->nav2_status_=4;
                RCLCPP_INFO(node_->get_logger(), "✅ 导航成功！");
            }
            else if (status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
                this->nav2_status_=6;
                RCLCPP_ERROR(node_->get_logger(), "❌ 导航失败/终止");
            }
            else if (status.status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
                this->nav2_status_=5;
                RCLCPP_WARN(node_->get_logger(), "⚠️ 导航被取消");
            }
            else if (status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
                this->nav2_status_=2;
                RCLCPP_INFO(node_->get_logger(), "🚀 导航执行中");
            }
        }
        // 打印更多信息
        // RCLCPP_INFO(node_->get_logger(), 
        //             "Goal ID: %s, Status: %d",
        //             msg->goal_id.c_str(),
        //             msg->status);
    }

    void FSM::decision(int is_game,int current_hp,int projectile_allowance){
        if(is_game){
            RCLCPP_INFO(node_->get_logger(), "game is not start");
            return ;
        }
        if(this->nav2_status_==2){
            //导航执行中
            RCLCPP_INFO(node_->get_logger(), "🚀 导航执行中, not pub goal");
            return;
        }
        decision::Point target_goal(0,0,0);
        if(current_hp<200||projectile_allowance<-1){
            target_goal=goal_point_sum_.home;
        }else{
            target_goal=goal_point_sum_.Patrol1;
            this->current_patrol_index_=1;
            //std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        // 检查是否重复发送相同目标
        // if (this->nav2_status_!=4&&std::abs(last_sent_goal_.x - target_goal.x) < 0.01 &&
        //     std::abs(last_sent_goal_.y - target_goal.y) < 0.01) {
        //     // 相同目标，不重复发送
        //     RCLCPP_INFO(node_->get_logger(), "相同目标，不重复发送");
        //     return;
        // }
        if(this->current_patrol_index_==1&&this->nav2_status_==4){
            this->current_patrol_index_==2;
            RCLCPP_INFO(node_->get_logger(),"go Patrol1");
            target_goal=goal_point_sum_.Patrol2;
            pub_goal(target_goal);
            this->last_sent_goal_ = target_goal;
            return ;
        }
        pub_goal(target_goal);
        this->last_sent_goal_ = target_goal;
    }
    void FSM::pub_goal(Point goal_point){
        auto msg = geometry_msgs::msg::PoseStamped();

        // 设置时间戳和坐标系
        msg.header.stamp = node_->now();
        msg.header.frame_id = this->map_tf_name_; //!!!

        // 设置位置
        msg.pose.position.x = goal_point.x;
        msg.pose.position.y = goal_point.y;
        msg.pose.position.z = 0.0;

        // 设置方向（四元数）
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;

        goal_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published goal: (%.2f, %.2f, %.2f rad)",
                    goal_point.x, goal_point.y, goal_point.yaw);
    }
}