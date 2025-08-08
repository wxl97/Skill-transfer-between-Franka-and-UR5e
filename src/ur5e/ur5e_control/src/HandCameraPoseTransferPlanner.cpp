// hand_camera_based_pose_transformer_planner.cpp
#include "ur5e_control/skill/HandCameraPoseTransferPlanner.hpp"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ur5e_control/ObjectPose.h>
#include <tf_conversions/tf_eigen.h>

#include <iostream>

HandCameraPoseTransformerPlanner::HandCameraPoseTransformerPlanner(): listener_(std::make_shared<tf::TransformListener>()) {
    ROS_INFO("PoseTransformerPlanner node initialized.");
}

HandCameraPoseTransformerPlanner::~HandCameraPoseTransformerPlanner() {}

geometry_msgs::Pose HandCameraPoseTransformerPlanner::getEndEffectorPose() const {
    const std::string baseFrame = "panda_link0";
    const std::string eeFrame = "panda_hand_tcp";

    geometry_msgs::Pose pose;

    try {
        // 检查 listener_ 是否有效
        if (!listener_) {
            ROS_ERROR("Transform listener is not initialized!");
            return pose;
        }
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

geometry_msgs::Pose HandCameraPoseTransformerPlanner::computeTargetPose() 
{
    geometry_msgs::Pose P_TCP_Obj = *ros::topic::waitForMessage<geometry_msgs::Pose>("/hand_cam_target_pose", ros::Duration(10.0));
    
    ROS_INFO("Get Object Pose related to TCP - x: %.4f, y: %.4f, z: %.4f",
             P_TCP_Obj.position.x, P_TCP_Obj.position.y, P_TCP_Obj.position.z);
            
    Eigen::Matrix4d T_TCP_Obj = PoseToMatrix(P_TCP_Obj);

    geometry_msgs::Pose current_pose = getEndEffectorPose();

    ROS_INFO("Current Position - (x: %.4f, y: %.4f, z: %.4f)",
             current_pose.position.x, current_pose.position.y, current_pose.position.z);

    Eigen::Matrix4d T_Base_TCP = PoseToMatrix(current_pose);

    Eigen::Matrix4d T_target = T_Base_TCP * T_TCP_Obj;

    geometry_msgs::Pose P_Base_Obj = MatrixToPose(T_target);

    ROS_INFO("Target Position - (x: %.4f, y: %.4f, z: %.4f)",
             P_Base_Obj.position.x, P_Base_Obj.position.y, P_Base_Obj.position.z);
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  Optimize the end-effector's rotation to the smallest angle using symmetry data from ObjectPose.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    zhang_franka_control::ObjectPose tagPose = *ros::topic::waitForMessage<zhang_franka_control::ObjectPose>("/tag_pose", ros::Duration(10.0));
    
    // T_Gripper_Cam, get from handeye calibration
    geometry_msgs::Pose P_TCP_Cam;
    P_TCP_Cam.position.x = 0.05385516850177642;
    P_TCP_Cam.position.y = -0.03320715252300718;
    P_TCP_Cam.position.z = -0.04688242842728654;
    P_TCP_Cam.orientation.x = 0.0194258121765898;
    P_TCP_Cam.orientation.y = 0.0027251998310264813;
    P_TCP_Cam.orientation.z = 0.679820530708974;
    P_TCP_Cam.orientation.w = 0.7331161280000117;
    Eigen::Matrix4d T_TCP_Cam = PoseToMatrix(P_TCP_Cam);

    Eigen::Matrix4d T_Cam_Tag = PoseToMatrix(tagPose.pose);
    Eigen::Matrix3d T_Cam_Tag_R = T_Cam_Tag.block<3,3>(0,0);
    Eigen::Vector3d eulerAngles = T_Cam_Tag_R.eulerAngles(2, 1, 0);  // ZYX

    double R_Z_Angle_Rad = eulerAngles[0];
    double R_Y_Angle_Rad = eulerAngles[1];
    double R_X_Angle_Rad = eulerAngles[2];

    double mini_Angle_Rad = 0.0;
    tf::Quaternion newOrientation(0.0, 0.0, 0.0, 1.0);
    tf::Quaternion currentOrientation(current_pose.orientation.x,current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    tf::Quaternion rot;
    
    if (tagPose.symmetry == "cubic") {
        mini_Angle_Rad = fmod(R_Z_Angle_Rad, M_PI / 2.0);
        if (mini_Angle_Rad > M_PI / 4.0) mini_Angle_Rad -= M_PI / 2.0;
        if (mini_Angle_Rad < -M_PI / 4.0) mini_Angle_Rad += M_PI / 2.0;
        rot.setRotation(tf::Vector3(0, 0, 1), -mini_Angle_Rad);
        newOrientation = rot * currentOrientation;
    } else if (tagPose.symmetry == "cuboid") {
        std::cout << "cuboid/cuboid///////////////////////////////////////" << R_Z_Angle_Rad << std::endl;
        if (R_Z_Angle_Rad <= M_PI / 2.0) mini_Angle_Rad = R_Z_Angle_Rad;
        if (R_Z_Angle_Rad > M_PI / 2.0) mini_Angle_Rad = R_Z_Angle_Rad - M_PI;
        // Eigen::Matrix4d T_Robot_Tag = T_Base_TCP * T_TCP_Cam * T_Cam_Tag;
        rot.setRotation(tf::Vector3(0, 0, 1), -mini_Angle_Rad);
        newOrientation = rot * currentOrientation;

        tf::Transform newTransform;
        newTransform.setRotation(newOrientation);
        tf::Vector3 localY = newTransform.getBasis() * tf::Vector3(0, 1, 0);  // 局部 y 轴
        double angle = -M_PI / 4.0;
        tf::Quaternion rotationDelta;
        rotationDelta.setRotation(localY.normalized(), angle);

        newOrientation = rotationDelta * newOrientation;
        newOrientation.normalize();
    }
    
    
    geometry_msgs::Pose target_pose = current_pose;
    target_pose.position.x = P_Base_Obj.position.x;
    target_pose.position.y = P_Base_Obj.position.y;
    target_pose.position.z = P_Base_Obj.position.z + 0.01;
    // Uncomment below to update orientation if needed
    target_pose.orientation.x = newOrientation.x();
    target_pose.orientation.y = newOrientation.y();
    target_pose.orientation.z = newOrientation.z();
    target_pose.orientation.w = newOrientation.w();

    return target_pose;
}

Eigen::Matrix4d HandCameraPoseTransformerPlanner::PoseToMatrix(const geometry_msgs::Pose& pose)
{
    // 提取平移部分
    Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
    
    // 提取并构造四元数（注意Eigen的构造参数顺序是w,x,y,z）
    Eigen::Quaterniond quat(pose.orientation.w, 
                     pose.orientation.x,
                     pose.orientation.y,
                     pose.orientation.z);
    
    // 构造4x4变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = quat.normalized().toRotationMatrix();
    T.block<3,1>(0,3) = translation;
    // 4. 检查矩阵有效性
    if (T.hasNaN()) {
        std::cerr << "Generated invalid matrix:\n" << T << std::endl;
        throw std::runtime_error("Matrix contains NaN!");
    }
    return T;
}

geometry_msgs::Pose HandCameraPoseTransformerPlanner::MatrixToPose(const Eigen::Matrix4d& T)
{
    // 提取平移部分
    geometry_msgs::Pose pose;
    Eigen::Vector3d translation = T.block<3,1>(0,3);
    pose.position.x = translation.x();
    pose.position.y = translation.y();
    pose.position.z = translation.z();

    // 提取旋转矩阵并转换为四元数
    Eigen::Matrix3d rotation = T.block<3,3>(0,0);
    Eigen::Quaterniond quat(rotation);
    quat.normalize();
    
    // 注意四元数存储顺序（Eigen返回的是w,x,y,z）
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    
    return pose;
}

