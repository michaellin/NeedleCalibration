#include "ros/ros.h"
#include "roboticArm.h"
#include "URcontrolV2.h"
//#include "tactilesensors4/StaticData.h"
#include <math.h>
#include <signal.h>

#include <iostream>
#include <fstream>

#define TEST
//#define EXPER

void my_handler(int s){
    ROS_INFO("Caught signal");
    ros::shutdown();
}


#ifdef EXPER
int main (int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle n; /* necessaire pour que ros::ok retourne true ??? */
    //ros::Subscriber sub_static = n.subscribe("/TactileSensor4/StaticData", 1000, staticdataCallback);

    //Robot control object
    RoboticArm robot;

    //Array of poses
    std::vector<tf::Transform> trajectory;

    //Poses
    tf::Transform tfpose_init, tfpose_prehomed, tfpose_homed, tfpose_oriented;
    tf::Transform tfpose_corner1, tfpose_corner2, tfpose_corner3, tfpose_corner4;
    tf::Transform tfpose_cwrot, tfpose_ccwrot;


    while(ros::ok())
    {
        //Tool offset
        tf::Vector3 tool_offset;
        tool_offset.setValue(0.0,0.0,0.0);

		ROS_INFO("READY TO HOME");
		getchar();

		//Home robot first
		robot.go_home();

		tfpose_init = robot.get_arm_position("base_link");

		//Calculate matrix to orient gripper towards table
		orientEffectorTowardsTable(tfpose_init, tfpose_oriented);

		//Rotate the end effector
		rotateGripperZ(0.0, tool_offset, tfpose_oriented, tfpose_prehomed);
		translateGripperX(0.009525, tfpose_prehomed, tfpose_homed);
		trajectory.clear();
		trajectory.push_back(tfpose_oriented);
		trajectory.push_back(tfpose_prehomed);
		trajectory.push_back(tfpose_homed);
        robot.execute_trajectory(trajectory, 0.1, "base_link");


        ROS_INFO("READY TO MOVE");
        getchar();

//      //Move to only one position
//      robot.move_to_point(tfpose_oriented,"base_link",0.1);

        //Calculate 4 poses to make a square;
        translateGripperZ(-0.15,tfpose_oriented,tfpose_corner1);
        translateGripperX(0.15,tfpose_corner1,tfpose_corner2);
        translateGripperZ(0.15,tfpose_corner2,tfpose_corner3);
        translateGripperX(-0.15,tfpose_corner3,tfpose_corner4);

        trajectory.clear();

        //Fill the array
        trajectory.push_back(tfpose_corner1);
        trajectory.push_back(tfpose_corner2);
        trajectory.push_back(tfpose_corner3);
        trajectory.push_back(tfpose_corner4);


        ROS_INFO("READY TO MOVE");
        getchar();

        /* execution*/
        robot.execute_trajectory(trajectory, 0.1, "base_link");



        ROS_INFO("PROGRAM COMPLETED");

        ros::spinOnce();
    }

}
#endif


#ifdef TEST
int main (int argc, char **argv)
{

	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);

    ros::init(argc, argv, "move_robot");
    ros::NodeHandle n; /* necessaire pour que ros::ok retourne true ??? */
    //ros::Subscriber sub_static = n.subscribe("/TactileSensor4/StaticData", 1000, staticdataCallback);

    //Robot control object
    RoboticArm robot;

    //Array of poses
    std::vector<tf::Transform> trajectory;

    //Poses
    tf::Transform tfpose_init, tfpose_homed, tfpose_oriented;
    tf::Transform tfpose_corner1, tfpose_corner2, tfpose_corner3, tfpose_corner4;
    tf::Transform tfpose_cwrot, tfpose_ccwrot;
    tf::Transform tfpose_temp1, tfpose_temp2, tfpose_temp3, tfpose_temp4, tfpose_temp5, tfpose_temp6, tfpose_temp7, tfpose_temp8, tfpose_temp9, tfpose_temp10;
    tf::Vector3 tool_offset;

    //For testing purposes
    tf::Transform tfpose_rotated;
    bool init_state = true;
    int input;
    bool rotated = true; // For testing purposes
    float xoffset = 0.0;
	float yoffset = -0.1599;
	float zoffset = -0.022;

    while(ros::ok())
    {
		if (init_state) {
			//Tool offset
			tool_offset.setValue(0.0,0.0,0.0);

			ROS_INFO("READY TO HOME");
			input = getchar();

			if (input = 'y') {
				//Home robot first
				robot.go_home();

				tfpose_init = robot.get_arm_position("base_link");

				//Calculate matrix to orient gripper towards table
				orientEffectorTowardsTable(tfpose_init, tfpose_oriented);

				//Rotate the end effector
				rotateGripperZ(0.0, tool_offset, tfpose_oriented, tfpose_temp1);
				translateGripperZ(0.0, tfpose_temp1, tfpose_temp2);
				translateGripperX(-0.4064, tfpose_temp2, tfpose_temp3);
				translateGripperY(0.15, tfpose_temp3, tfpose_temp4);
				rotateGripperY(M_PI/4, tool_offset, tfpose_temp4, tfpose_homed);
				//translateGripperX(0.009525, tfpose_prehomed, tfpose_homed);
				trajectory.clear();
				trajectory.push_back(tfpose_oriented);
				trajectory.push_back(tfpose_temp1);
				trajectory.push_back(tfpose_temp2);
				trajectory.push_back(tfpose_temp3);
				trajectory.push_back(tfpose_temp4);
				trajectory.push_back(tfpose_homed);
				robot.execute_trajectory(trajectory, 0.1, "base_link");
			}
			init_state = false;
			tool_offset.setValue(xoffset, yoffset, zoffset);
			ROS_INFO("READY TO MOVE");
		}



		input = getchar();
		if (input == 'c') {
			tool_offset.setValue(xoffset, yoffset, zoffset);
			if (rotated) {
				rotated = false;
				rotateGripperY(-M_PI/20, tool_offset, tfpose_homed, tfpose_temp1);
				rotateGripperY(-M_PI/20, tool_offset, tfpose_temp1, tfpose_temp2);
				rotateGripperY(-M_PI/20, tool_offset, tfpose_temp2, tfpose_temp3);
				rotateGripperY(-M_PI/20, tool_offset, tfpose_temp3, tfpose_temp4);
				rotateGripperY(-M_PI/20, tool_offset, tfpose_temp4, tfpose_temp5);
				rotateGripperY(-M_PI/20, tool_offset, tfpose_temp5, tfpose_temp6);
				rotateGripperY(-M_PI/20, tool_offset, tfpose_temp6, tfpose_temp7);
				rotateGripperY(-M_PI/20, tool_offset, tfpose_temp7, tfpose_temp8);
				rotateGripperY(-M_PI/20, tool_offset, tfpose_temp8, tfpose_temp9);
				rotateGripperY(-M_PI/20, tool_offset, tfpose_temp9, tfpose_rotated);
				trajectory.clear();
				trajectory.push_back(tfpose_temp1);
				trajectory.push_back(tfpose_temp2);
				trajectory.push_back(tfpose_temp3);
				trajectory.push_back(tfpose_temp4);
				trajectory.push_back(tfpose_temp5);
				trajectory.push_back(tfpose_temp6);
				trajectory.push_back(tfpose_temp7);
				trajectory.push_back(tfpose_temp8);
				trajectory.push_back(tfpose_temp9);
				trajectory.push_back(tfpose_rotated);
				robot.execute_trajectory(trajectory, 0.1, "base_link");
				ROS_INFO("rotate wrist");
			} else {
				rotated = true;
				rotateGripperY(M_PI/20, tool_offset, tfpose_rotated, tfpose_temp1);
				rotateGripperY(M_PI/20, tool_offset, tfpose_temp1, tfpose_temp2);
				rotateGripperY(M_PI/20, tool_offset, tfpose_temp2, tfpose_temp3);
				rotateGripperY(M_PI/20, tool_offset, tfpose_temp3, tfpose_temp4);
				rotateGripperY(M_PI/20, tool_offset, tfpose_temp4, tfpose_temp5);
				rotateGripperY(M_PI/20, tool_offset, tfpose_temp5, tfpose_temp6);
				rotateGripperY(M_PI/20, tool_offset, tfpose_temp6, tfpose_temp7);
				rotateGripperY(M_PI/20, tool_offset, tfpose_temp7, tfpose_temp8);
				rotateGripperY(M_PI/20, tool_offset, tfpose_temp8, tfpose_temp9);
				rotateGripperY(M_PI/20, tool_offset, tfpose_temp9, tfpose_homed);
				trajectory.clear();
				trajectory.push_back(tfpose_temp1);
				trajectory.push_back(tfpose_temp2);
				trajectory.push_back(tfpose_temp3);
				trajectory.push_back(tfpose_temp4);
				trajectory.push_back(tfpose_temp5);
				trajectory.push_back(tfpose_temp6);
				trajectory.push_back(tfpose_temp7);
				trajectory.push_back(tfpose_temp8);
				trajectory.push_back(tfpose_temp9);
				trajectory.push_back(tfpose_homed);
				robot.execute_trajectory(trajectory, 0.1, "base_link");
				ROS_INFO("unrotate wrist");
			}
		} 
////	} else if (input == 'w') {
////		yoffset += 0.0001;
////		ROS_INFO("yoffset at %f", yoffset);
////	} else if (input == 's') {
////		yoffset -= 0.0001;
////		ROS_INFO("yoffset at %f", yoffset);
////	} else if (input == 'a') {
////		xoffset += 0.001;
////		ROS_INFO("xoffset at %f", xoffset);
////	} else if (input == 'd') {
////		xoffset -= 0.001;
////		ROS_INFO("xoffset at %f", xoffset);
////	} else if (input == 'r') {
////		zoffset -= 0.0001;
////		ROS_INFO("zoffset at %f", zoffset);
////	} else if (input == 'f') {
////		zoffset += 0.0001;
////		ROS_INFO("zoffset at %f", zoffset);
////	}

        //ros::spinOnce();
    }
}
#endif
