#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ur5e_control/MultiObjectPose.h>
#include <Eigen/Geometry>
#include "ur5e_control/BaseCameraPoseTransferPlanner.hpp"

BaseCameraPoseTransformerPlanner::BaseCameraPoseTransformerPlanner()
    : listener_(std::make_shared<tf::TransformListener>()) {
    ROS_INFO("PoseTransformerPlanner initialized.");
}

geometry_msgs::Pose BaseCameraPoseTransformerPlanner::getEndEffectorPose() const {
    const std::string baseFrame = "panda_link0";
    const std::string eeFrame = "panda_hand_tcp";

    geometry_msgs::Pose pose;

    try {
        listener_->waitForTransform(baseFrame, eeFrame, ros::Time(0), ros::Duration(4.0));
        tf::StampedTransform transform;
        listener_->lookupTransform(baseFrame, eeFrame, ros::Time(0), transform);

        pose.position.x = transform.getOrigin().x();
        pose.position.y = transform.getOrigin().y();
        pose.position.z = transform.getOrigin().z();

        tf::Quaternion q = transform.getRotation();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    } catch (const tf::TransformException& ex) {
        ROS_ERROR("Failed to get transform: %s", ex.what());
    }

    return pose;
}

geometry_msgs::Pose BaseCameraPoseTransformerPlanner::computeTargetPose(const std::string& objectName) const {
    geometry_msgs::Pose targetPose;

    auto msg = ros::topic::waitForMessage<zhang_franka_control::MultiObjectPose>("/base_cam_target_pose");

    if (!msg) {
        ROS_ERROR("No message received on /base_cam_target_pose");
        return targetPose;
    }

    geometry_msgs::Pose objectPose;
    bool found = false;
    for (const auto& obj : msg->objects) {
        if (obj.name == objectName) {
            objectPose = obj.pose;
            found = true;

            ROS_INFO("Found object '%s' at (%.3f, %.3f, %.3f)",
                     objectName.c_str(),
                     objectPose.position.x, objectPose.position.y, objectPose.position.z);
            break;
        }
    }

    if (!found) {
        ROS_WARN("Object '%s' not found in pose list", objectName.c_str());
        return targetPose;
    }

    const auto currentPose = getEndEffectorPose();

    // 设置目标位置，带一定偏移量
    targetPose.position.x = objectPose.position.x - 0.05;
    targetPose.position.y = objectPose.position.y;
    targetPose.position.z = 0.15;

    // 设置目标朝向，绕Z轴旋转 -15°
    const double angleRad = -15.0 * M_PI / 180.0;
    tf::Quaternion rot;
    rot.setRotation(tf::Vector3(0, 0, 1), angleRad);

    tf::Quaternion baseOrientation(1, 0, 0, 0);
    tf::Quaternion newOrientation = rot * baseOrientation;

    targetPose.orientation.x = newOrientation.x();
    targetPose.orientation.y = newOrientation.y();
    targetPose.orientation.z = newOrientation.z();
    targetPose.orientation.w = newOrientation.w();

    ROS_INFO("Computed Target Pose (x=%.3f, y=%.3f, z=%.3f)",
             targetPose.position.x, targetPose.position.y, targetPose.position.z);
    ROS_INFO("Orientation (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
             targetPose.orientation.x, targetPose.orientation.y,
             targetPose.orientation.z, targetPose.orientation.w);

    return targetPose;
}