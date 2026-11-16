#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>

#include <iostream>

#include <cmath>

#include <cstdlib>

#include <algorithm>



const float Tr_m[] = {1.0, 4.5, 4.5};

const float R2_L1 = 0.224, R2_L2 = 0.250, R2_L3 = 0.230, R2_L4 = 0.171;

const float LR2 = -0.020, WR2 = 0.460 , HR2 = 0.285;

const float R2_ori = 0.0;

const int stepsPerRevolution = 6400;



void fix_point(float px, float py, float pz, float &px_R2, float &py_R2, float &pz_R2) {

    px_R2 = px + LR2;

    py_R2 = py + WR2;

    pz_R2 = pz + HR2;

}



void printUsage() {

    ROS_INFO("\n========================================================");

    ROS_INFO("Usage Guide:");

    ROS_INFO(" 1- Forward Kinematics:");

    ROS_INFO("    rosrun <package_name> <node_name> 1 <th1> <th2> <th3> <th4> <gripper_state>");

    ROS_INFO("\n 2- Inverse Kinematics:");

    ROS_INFO("    rosrun <package_name> <node_name> 2 <Px> <Py> <Pz> <gripper_state>");

    ROS_INFO("========================================================\n");

}



int main(int argc, char **argv) {

    ros::init(argc, argv, "robot_controller");

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



        // حساب الخطوات كـ float لضمان عدم ضياع أي أجزاء عشرية أثناء النقل

        for (int i = 0; i < 3; i++) {

            msg.data[i] = (angles[i] / 360.0f) * stepsPerRevolution * Tr_m[i];

        }

        msg.data[3] = std::max(0.0f, std::min(180.0f, angles[3]));

        msg.data[4] = (gripper == 1.0f) ? 1.0f : 0.0f;



        ROS_INFO("Executing Forward Kinematics with Float steps...");



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

        float px_R2, py_R2, pz_R2;

        fix_point(px, py, pz, px_R2, py_R2, pz_R2);



        float R2_q1 = std::atan2(py_R2, px_R2);

        if (py_R2 == 0 && px_R2 == 0) R2_q1 = 0;



        float A = px_R2 - R2_L4 * std::cos(R2_q1) * std::cos(R2_ori);

        float B = py_R2 - R2_L4 * std::sin(R2_q1) * std::cos(R2_ori);

        float C = pz_R2 - R2_L1 - R2_L4 * std::sin(R2_ori);



        float cos_q3 = (A*A + B*B + C*C - R2_L2*R2_L2 - R2_L3*R2_L3) / (2 * R2_L2 * R2_L3);



        if (cos_q3 > 1.0 || cos_q3 < -1.0) {

            ROS_ERROR("Error: Target Coordinates Out of Reach!");

            return 1;

        }



        float R2_q3 = -std::acos(cos_q3);

        float a = R2_L3 * std::sin(R2_q3);

        float b = R2_L2 + R2_L3 * std::cos(R2_q3);

        float r = std::sqrt(a*a + b*b);



        float R2_q2 = std::atan2(C, std::sqrt(r*r - C*C)) - std::atan2(a, b);

        float R2_q4 = R2_ori - R2_q2 - R2_q3;

        R2_q2 = M_PI - R2_q2;

        R2_q3 = -M_PI/2.0 - (-R2_q3 - 2 * 135 * M_PI / 180.0) - R2_q2;

        

        float angles_deg[4];

        angles_deg[0] = R2_q1 * 180.0 / M_PI;

        angles_deg[1] = R2_q2 * 180.0 / M_PI;

        angles_deg[2] = R2_q3 * 180.0 / M_PI;

        angles_deg[3] = R2_q4 * 180.0 / M_PI;



        for (int i = 0; i < 3; i++) {

            msg.data[i] = (angles_deg[i] / 360.0f) * stepsPerRevolution * Tr_m[i];

        }

        msg.data[3] = std::max(0.0f, std::min(180.0f, angles_deg[3]));

        msg.data[4] = (gripper == 1.0f) ? 1.0f : 0.0f;



        ROS_INFO("Executing Inverse Kinematics with Float steps...");



    } else {

        ROS_ERROR("Invalid choice!");

        printUsage();

        return 1;

    }



    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("robot_commands", 10);



    ROS_INFO("Connecting to Arduino...");

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



    ROS_INFO("Commands Sent Successfully. Exiting Node.");

    return 0;

}
