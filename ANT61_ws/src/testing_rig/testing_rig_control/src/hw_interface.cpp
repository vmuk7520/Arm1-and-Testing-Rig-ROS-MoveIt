#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <thread>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <cmath>

#include "dynamixel_sdk_examples/SyncGetPosition.h"
#include "dynamixel_sdk_examples/SyncSetPosition.h"

class Testing_rig : public hardware_interface::RobotHW {
public:
    Testing_rig(ros::NodeHandle& nh) : nh_(nh) {

        // Connect and register the joint state and position interfaces
        hardware_interface::JointStateHandle state_handle_outer("outer_ring", &pos_[0], &vel_[0], &eff_[0]);
        jnt_state_interface_.registerHandle(state_handle_outer);

        hardware_interface::JointStateHandle state_handle_middle("middle_ring", &pos_[1], &vel_[1], &eff_[1]);
        jnt_state_interface_.registerHandle(state_handle_middle);

        hardware_interface::JointStateHandle state_handle_inner("inner_ring", &pos_[2], &vel_[2], &eff_[2]);
        jnt_state_interface_.registerHandle(state_handle_inner);

        registerInterface(&jnt_state_interface_);

        hardware_interface::JointHandle pos_handle_outer(jnt_state_interface_.getHandle("outer_ring"), &cmd_[0]);
        jnt_pos_interface_.registerHandle(pos_handle_outer);

        hardware_interface::JointHandle pos_handle_middle(jnt_state_interface_.getHandle("middle_ring"), &cmd_[1]);
        jnt_pos_interface_.registerHandle(pos_handle_middle);

        hardware_interface::JointHandle pos_handle_inner(jnt_state_interface_.getHandle("inner_ring"), &cmd_[2]);
        jnt_pos_interface_.registerHandle(pos_handle_inner);

        registerInterface(&jnt_pos_interface_);

        // setup subscriber and publisher
        client = nh.serviceClient<dynamixel_sdk_examples::SyncGetPosition>("/sync_get_position");
        cmd_publisher_ = nh.advertise<dynamixel_sdk_examples::SyncSetPosition>("/sync_set_position", 10);
        joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    }

    // Publish command args for CAN_tx node
    void write() {
        dynamixel_sdk_examples::SyncSetPosition msg;
        
        float conversion = 0.087891;
        int MAX = 4095;
        
        float radians[3];
        float degrees[3];
        float position[3];
        
        for (int i = 0; i < 3; i++) {
            float positive_radians = std::abs(cmd_[i]);

            // Calculate the number of full rotations (multiples of 2π)
            int rotations = static_cast<int>(positive_radians / (2.0f * M_PI));

            // Wrap-around using modulo
            radians[i] = positive_radians - rotations * (2.0f * M_PI);

            if (cmd_[i] < 0){
                radians[i] = 2*M_PI - radians[i];
            }
        }

        // std::cout << "radians: " << radians[0] << " " << radians[1] << " " << radians[2] << std::endl;

        degrees[0] = (radians[0] * 180.0 / M_PI)+180;
        degrees[1] = (radians[1] * 180.0 / M_PI);
        degrees[2] = (radians[2] * 180.0 / M_PI)+90;

        // std::cout << "degrees: " << degrees[0] << " " << degrees[1] << " " << degrees[2] << std::endl;

        for (int i = 0; i < 3; i++) {
            position[i] = degrees[i] / conversion;
            if (position[i] > MAX){
                position[i] = position[i] - MAX;
            }
        }

        msg.id0 = 0;
        msg.id1 = 1;
        msg.id2 = 2;
        msg.position0 = position[0];
        msg.position1 = position[1];
        msg.position2 = position[2];

        cmd_publisher_.publish(msg);
  }

    void read(){
        
        float offset = 90;
        float conversion = 0.087891;

        dynamixel_sdk_examples::SyncGetPosition srv;

        srv.request.id0 = 0;
        srv.request.id1 = 1;
        srv.request.id2 = 2;

        if (client.call(srv)) {
            float degrees[3];
            degrees[0] = (srv.response.position0 * conversion)-180;
            degrees[1] = (srv.response.position1 * conversion);
            degrees[2] = (srv.response.position2 * conversion)-90;
            for (int i = 0; i < 3; i++) {
                pos_[i] = degrees[i] * M_PI / 180.0;
            }
        } else {
            ROS_ERROR_STREAM("Failed to call service /get_position");
            }
        
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name = {"outer_ring", "middle_ring", "inner_ring"};
        joint_state_msg.position = {pos_[0], pos_[1], pos_[2]};

        joint_state_msg.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state_msg);
        
        }

    void controlLoop(controller_manager::ControllerManager& cm_ref) {
        ros::Rate rate(50); // 50 Hz control loop
        while (ros::ok()) {
            read();
            cm_ref.update(ros::Time::now(), ros::Duration(0.02));
            write();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;

    double cmd_[3];
    double pos_[3];
    double vel_[3];
    double eff_[3];

    double prev_cmd[3] = {0.0,0.0,0.0};

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;

    ros::ServiceClient client;
    ros::Publisher cmd_publisher_;
    ros::Publisher joint_pub;

    // joint_state callback from CAN_rx node
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
        // Update joint positions based on received message
        if (msg->name.size() >= 3) {
            for (int i = 0; i < 3; i++) {
                pos_[i] = msg->position[i];
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm1_hardware_interface");
    ros::NodeHandle nh;

    Testing_rig my_robot(nh);
    controller_manager::ControllerManager cm(&my_robot, nh);

    // Start the control loop in a separate thread
    std::thread control_thread(&Testing_rig::controlLoop, &my_robot, std::ref(cm));

    // Use AsyncSpinner for tasks as service callbacks can block the main loop
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Main thread spins once
    while (ros::ok()) {
        ros::spinOnce();
    }

    spinner.stop();
    control_thread.join();

    return 0;
}