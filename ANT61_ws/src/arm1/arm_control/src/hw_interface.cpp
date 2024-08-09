#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <thread>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

class Arm1 : public hardware_interface::RobotHW {
public:
    Arm1(ros::NodeHandle& nh) : nh_(nh) {

        // Connect and register the joint state and position interfaces
        hardware_interface::JointStateHandle state_handle_shoulder("shoulder", &pos_[0], &vel_[0], &eff_[0]);
        jnt_state_interface_.registerHandle(state_handle_shoulder);

        hardware_interface::JointStateHandle state_handle_elbow1("elbow_1", &pos_[1], &vel_[1], &eff_[1]);
        jnt_state_interface_.registerHandle(state_handle_elbow1);

        hardware_interface::JointStateHandle state_handle_elbow2("elbow_2", &pos_[2], &vel_[2], &eff_[2]);
        jnt_state_interface_.registerHandle(state_handle_elbow2);

        hardware_interface::JointStateHandle state_handle_wrist("wrist", &pos_[3], &vel_[3], &eff_[3]);
        jnt_state_interface_.registerHandle(state_handle_wrist);

        registerInterface(&jnt_state_interface_);

        hardware_interface::JointHandle pos_handle_shoulder(jnt_state_interface_.getHandle("shoulder"), &cmd_[0]);
        jnt_pos_interface_.registerHandle(pos_handle_shoulder);

        hardware_interface::JointHandle pos_handle_elbow1(jnt_state_interface_.getHandle("elbow_1"), &cmd_[1]);
        jnt_pos_interface_.registerHandle(pos_handle_elbow1);

        hardware_interface::JointHandle pos_handle_elbow2(jnt_state_interface_.getHandle("elbow_2"), &cmd_[2]);
        jnt_pos_interface_.registerHandle(pos_handle_elbow2);

        hardware_interface::JointHandle pos_handle_wrist(jnt_state_interface_.getHandle("wrist"), &cmd_[3]);
        jnt_pos_interface_.registerHandle(pos_handle_wrist);

        registerInterface(&jnt_pos_interface_);

        // setup subscriber and publisher
        joint_state_subscriber_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &Arm1::jointStateCallback, this);
        cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("/arm_commands", 10);
    }

    // Publish command args for CAN_tx node
    void write() {
        geometry_msgs::Twist msg;
        msg.linear.x = cmd_[0];
        msg.linear.y = cmd_[1]; 
        msg.linear.z = cmd_[2]; 
        msg.angular.z = cmd_[3]; 
        cmd_publisher_.publish(msg);
    }

    void controlLoop(controller_manager::ControllerManager& cm_ref) {
        ros::Rate rate(50); // 50 Hz control loop
        while (ros::ok()) {
            cm_ref.update(ros::Time::now(), ros::Duration(0.02));
            write();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;

    double cmd_[4];
    double pos_[4];
    double vel_[4];
    double eff_[4];

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;

    ros::Subscriber joint_state_subscriber_;
    ros::Publisher cmd_publisher_;

    // joint_state callback from CAN_rx node
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
        // Update joint positions based on received message
        if (msg->name.size() >= 4) {
            for (int i = 0; i < 4; i++) {
                pos_[i] = msg->position[i];
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm1_hardware_interface");
    ros::NodeHandle nh;

    Arm1 my_robot(nh);
    controller_manager::ControllerManager cm(&my_robot, nh);

    // Start the control loop in a separate thread
    std::thread control_thread(&Arm1::controlLoop, &my_robot, std::ref(cm));

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