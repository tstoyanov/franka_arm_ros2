#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include<tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TestSM : public rclcpp::Node
{
  public:
    TestSM()
    : Node("test_state")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/cartesian_impedance_controller/equilibrium_pose", 10);
      publisher_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("equilibrium_pose_viz", 10);
      publisher_stiffness_ = this->create_publisher<geometry_msgs::msg::Twist>("/cartesian_impedance_controller/desired_stiffness", 10);
      timer_ = this->create_wall_timer(
          500ms, std::bind(&TestSM::timer_callback, this));
     
      //NOTE: controlled point is hand_tcp 
      geometry_msgs::msg::Pose current_target_pose;
      current_target_pose.position.x = 0.3;
      current_target_pose.position.y = -0.01;
      current_target_pose.position.z = 0.37;
      current_target_pose.orientation.x = 1.0;
      current_target_pose.orientation.y = 0.0;
      current_target_pose.orientation.z = 0.0;
      current_target_pose.orientation.w = 0.0;
      current_target_pose_array.push_back(current_target_pose);

      current_target_pose.position.x = 0.4;
//      current_target_pose.position.x = 0.25;
//      current_target_pose.position.y = 0.25;
//      current_target_pose.position.z = 0.45;
 //     current_target_pose.orientation.x = 0.737;
 //     current_target_pose.orientation.y = -0.669;
 //     current_target_pose.orientation.z = 0.11;
//      current_target_pose.orientation.w = -0.10;
      current_target_pose_array.push_back(current_target_pose);

      current_target_pose.position.z = 0.2;
//      current_target_pose.position.x = 0.45;
//      current_target_pose.position.y = 0.0;
//      current_target_pose.position.z = 0.2;
//      current_target_pose.orientation.x = -0.56;
//      current_target_pose.orientation.y = 0.81;
//      current_target_pose.orientation.z = 0.01;
//      current_target_pose.orientation.w = 0.13;
      current_target_pose_array.push_back(current_target_pose);

      geometry_msgs::msg::Twist t;
      t.linear.x=1900;
      t.linear.y=1900;
      t.linear.z=1900;
      t.angular.x=600;
      t.angular.y=600;
      t.angular.z=600;
      current_stiffness_array.push_back(t);

      t.linear.x=200;
      t.linear.y=200;
      t.linear.z=600;
      t.angular.x=600;
      t.angular.y=600;
      t.angular.z=600;
      current_stiffness_array.push_back(t);

      t.linear.x=600;
      t.linear.y=600;
      t.linear.z=10;
      t.angular.x=100;
      t.angular.y=100;
      t.angular.z=600;
      current_stiffness_array.push_back(t);

      idx=0;
      
      //set up tf_ listner
      tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

  private:
    void timer_callback()
    {
      publisher_->publish(current_target_pose_array[idx]);
      publisher_stiffness_->publish(current_stiffness_array[idx]);
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose=current_target_pose_array[idx];
      target_pose.header.frame_id="panda_link0";
      publisher_stamped_->publish(target_pose);
      
      std::string toFrame = "panda_link0";
      std::string fromFrame = "panda_hand_tcp";

      geometry_msgs::msg::TransformStamped t;

      // Look up for the transformation between target_frame and turtle2 frames
      // and send velocity commands for turtle2 to reach target_frame
      try {
        t = tf_buffer_->lookupTransform(
            toFrame, fromFrame,
            tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrame.c_str(), fromFrame.c_str(), ex.what());
        return;
      }

      Eigen::Affine3d current_pose = tf2::transformToEigen(t);
      Eigen::Affine3d desired_pose; 
      tf2::fromMsg(current_target_pose_array[idx],desired_pose);

      //std::cerr<<"current: "<<desired_pose.matrix()<<std::endl;
      //std::cerr<<"desired: "<<current_pose.matrix()<<std::endl;
      desired_pose = desired_pose.inverse()*current_pose;
      auto rot_part = Eigen::AngleAxisd(desired_pose.rotation());
      if(desired_pose.translation().norm()< 0.1 && rot_part.angle() < 0.5) {
        idx = (idx+1)%current_target_pose_array.size();
      } else {
        //std::cerr<<"diff to desired: "<<desired_pose.matrix();
        std::cerr<<"\ntranslation "<<desired_pose.translation().norm() << " rot "<<rot_part.angle()<<std::endl;
      }

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_stiffness_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_stamped_;
    std::vector<geometry_msgs::msg::Pose> current_target_pose_array;
    std::vector<geometry_msgs::msg::Twist> current_stiffness_array;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    int idx;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSM>());
  rclcpp::shutdown();
  return 0;
}
