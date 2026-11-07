#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <algorithm>

// --- ثوابت الذراع R1 ---
const float Tr_m[] = {3.3, 5.0, 3.5, 2.3};  
const float R1_L1 = 0.120, R1_L2 = 0.150, R1_L3 = 0.110, R1_L4 = 0.000;
const float LR1 = 0.000, WR1 = 0.310, HR1 = 0.000;
const float R1_ori = 0.0;
const int stepsPerRevolution = 6400;

void fix_point(float px, float py, float pz, float &px_R1, float &py_R1, float &pz_R1) {
    px_R1 = -py + LR1;
    py_R1 = px + WR1;
    pz_R1 = pz + HR1;
}

void printUsage() {
    ROS_INFO("\n========================================================");
    ROS_INFO("Usage Guide for Robot R1:");
    ROS_INFO(" 1- Forward Kinematics:");
    ROS_INFO("    rosrun <package_name> <node_name> 1 <th1> <th2> <th3> <th4> <gripper_state>");
    ROS_INFO("\n 2- Inverse Kinematics:");
    ROS_INFO("    rosrun <package_name> <node_name> 2 <Px> <Py> <Pz> <gripper_state>");
    ROS_INFO("========================================================\n");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "r1_robot_controller");
    ros::NodeHandle nh;

    if (argc < 2) {
        printUsage();
        return 1;
    }

    int choice = std::atoi(argv[1]);
    std_msgs::Float32MultiArray msg;
    msg.data.resize(5, 0.0f);

    if (choice == 1) {
        if (argc != 7) {
            ROS_ERROR("Error: Forward Mode requires 5 parameters.");
            printUsage();
            return 1;
        }

        float angles[4];
        for (int i = 0; i < 4; i++) {
            angles[i] = std::atof(argv[i + 2]);
        }
        float gripper = std::atof(argv[6]);

        // حساب الخطوات كـ float لضمان عدم ضياع أي أجزاء عشرية أثناء النقل لـ R1 (4 محركات)
        for (int i = 0; i < 4; i++) {
            msg.data[i] = (angles[i] / 360.0f) * stepsPerRevolution * Tr_m[i];
        }
        msg.data[3] = std::max(0.0f, std::min(180.0f, angles[3]));
        msg.data[4] = (gripper == 1.0f) ? 1.0f : 0.0f;

        ROS_INFO("Executing Forward Kinematics for R1 with Float steps...");

    } else if (choice == 2) {
        if (argc != 6) {
            ROS_ERROR("Error: Inverse Mode requires 4 parameters.");
            printUsage();
            return 1;
        }

        float px = std::atof(argv[2]);
        float py = std::atof(argv[3]);
        float pz = std::atof(argv[4]);
        float gripper = std::atof(argv[5]);
        float px_R1, py_R1, pz_R1;
        fix_point(px, py, pz, px_R1, py_R1, pz_R1);

        float R1_q1 = std::atan2(py_R1, px_R1);
        if (py_R1 == 0 && px_R1 == 0) R1_q1 = 0;

        float A = px_R1 - R1_L4 * std::cos(R1_q1) * std::cos(R1_ori);
        float B = py_R1 - R1_L4 * std::sin(R1_q1) * std::cos(R1_ori);
        float C = pz_R1 - R1_L1 - R1_L4 * std::sin(R1_ori);

        float cos_q3 = (A * A + B * B + C * C - R1_L2 * R1_L2 - R1_L3 * R1_L3) / (2.0f * R1_L2 * R1_L3);

        if (cos_q3 > 1.0f || cos_q3 < -1.0f) {
            ROS_ERROR("Error: Target Coordinates Out of Reach for R1!");
            return 1;
        }

        float R1_q3 = -std::acos(cos_q3);
        float a = R1_L3 * std::sin(R1_q3);
        float b = R1_L2 + R1_L3 * std::cos(R1_q3);
        float r = std::sqrt(a * a + b * b);

        float R1_q2 = std::atan2(C, std::sqrt(std::max(0.0f, r * r - C * C))) - std::atan2(a, b);
        float R1_q4 = R1_ori - R1_q2 - R1_q3;

        // التعديلات الهندسية الخاصة بـ R1
        R1_q3 = R1_q3 - std::atan2((R1_L2 * std::cos(R1_q2)), (R1_L2 * std::sin(R1_q2)));
        R1_q2 = M_PI - R1_q2;

        float angles_deg[4];
        angles_deg[0] = R1_q1 * 180.0 / M_PI;
        angles_deg[1] = R1_q2 * 180.0 / M_PI;
        angles_deg[2] = R1_q3 * 180.0 / M_PI;
        angles_deg[3] = R1_q4 * 180.0 / M_PI;

        for (int i = 0; i < 4; i++) {
            msg.data[i] = (angles_deg[i] / 360.0f) * stepsPerRevolution * Tr_m[i];
        }
        msg.data[3] = std::max(0.0f, std::min(180.0f, angles_deg[3]));
        msg.data[4] = (gripper == 1.0f) ? 1.0f : 0.0f;

        ROS_INFO("Executing Inverse Kinematics for R1 with Float steps...");

    } else {
        ROS_ERROR("Invalid choice for R1!");
        printUsage();
        return 1;
    }

    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("robot_commands", 10);

    ROS_INFO("Connecting to Arduino for R1...");
    while (pub.getNumSubscribers() < 1 && ros::ok()) {
        ros::WallDuration(0.2).sleep();
    }

    ros::Rate loop_rate(10);
    int publish_count = 0;

    while (ros::ok() && publish_count < 5) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        publish_count++;
    }

    ROS_INFO("Commands Sent Successfully for R1. Exiting Node.");
    return 0;
}
