#include "ros/ros.h"
#include "roboticArm.h"
#include "URcontrolV2.h"
//#include "tactilesensors4/StaticData.h"
#include <math.h>
#include <signal.h>

#include <actionlib/client/simple_action_client.h>

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
		robot.go_home(client);

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
////const float calibX[10] {
////0.0,
////0.04, 0.08, 0.12,
////0.0, 0.0, 0.0,
////0.0, 0.0, 0.0,
////};

////const float calibY[10] {
////-0.05,
////-0.05, -0.05, -0.05,
////-0.01, 0.03, 0.07,
////-0.05, -0.05, -0.05,
////};

////const float calibZ[10] {
////-0.15,
////-0.15, -0.15, -0.15,
////-0.15, -0.15, -0.15,
////-0.11, -0.07, -0.03,
////};
const int calibSteps = 36;
const float calibX[calibSteps] {
0.120, 0.10, 0.05,
0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
0.10, 0.08, 0.05,
0.10, 0.08, 0.05,
-0.10, -0.08, -0.05,
-0.10, -0.08, -0.05,
0.10, 0.08, 0.05,
0.10, 0.08, 0.05, // first one in row was not done for 11AM_03_09
-0.10, -0.08, -0.05,
-0.10, -0.08, -0.05,
};

const float calibY[calibSteps] {
0.00, 0.00, 0.00,
0.00, 0.00, 0.00,
-0.05, -0.10, -0.12,
0.01, 0.03, 0.00,
-0.05, -0.10, -0.12,
0.01, 0.03, 0.00,
-0.05, -0.10, -0.12,
0.01, 0.03, 0.00,
-0.05, -0.10, -0.12,
0.01, 0.03, 0.00, // first one in row was not done for 11AM_03_09
-0.05, -0.10, -0.12,
0.01, 0.03, 0.00,
};

const float calibZ[calibSteps] {
0.00, 0.00, 0.00,
-0.120, -0.10, -0.05,
0.00, -0.05, -0.10,
0.00, 0.00, -0.05,
0.00, -0.05, -0.10,
0.00, 0.00, 0.00,
-0.05, -0.05, -0.10,
0.00, 0.00, 0.00,
-0.140, -0.155, -0.150,
-0.140, -0.140, -0.140, // first one in row was not done for 11AM_03_09
-0.155, -0.155, -0.150,
-0.140, -0.140, -0.140,
};

const float positionX[40] {
	0.180, 0.140, 0.100, 0.060, 0.020,
	-0.020, -0.060, -0.100, -0.140, -0.180,
	0.180, 0.140, 0.100, 0.060, 0.020,
	-0.020, -0.060, -0.100, -0.140, -0.180,
	0.180, 0.140, 0.100, 0.060, 0.020,
	-0.020, -0.060, -0.100, -0.140, -0.180,
	0.180, 0.140, 0.100, 0.060, 0.020,
	-0.020, -0.060, -0.100, -0.140, -0.180,
};

const float positionZ[40] {
	-0.16, -0.16, -0.16, -0.16, -0.16,
	-0.16, -0.16, -0.16, -0.16, -0.16,
	-0.08, -0.08, -0.08, -0.08, -0.08,
	-0.08, -0.08, -0.08, -0.08, -0.08,
	-0.04, -0.04, -0.04, -0.04, -0.04,
	-0.04, -0.04, -0.04, -0.04, -0.04,
	0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0,
};

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
    tf::Transform tfpose_init, tfpose_homed, tfpose_oriented, tfpose_goal;
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
	float zoffset = -0.021;

	//For data saving purposes
	std::ofstream output_file;
	int stepNum = 0;
	int calibNum = 0;

    Client client ("follow_cart_trajectory");
    client.waitForServer();

    while(ros::ok())
    {
		if (init_state) {
			//Tool offset
			tool_offset.setValue(0.0,0.0,0.0);

			ROS_INFO("READY TO HOME");
			input = getchar();

			if (input = 'y') {
				//Home robot first
				robot.go_home(client);
				//robot.move_to_point(robot.tf_home, "base_link", 0.1, client);

				tfpose_init = robot.get_arm_position("base_link");

				//Calculate matrix to orient gripper towards table
				orientEffectorTowardsTable(tfpose_init, tfpose_oriented);

				//Rotate the end effector
				rotateGripperZ(0.0, tool_offset, tfpose_oriented, tfpose_temp1);
				translateGripperZ(0.135, tfpose_temp1, tfpose_temp2);
				translateGripperX(-0.4164, tfpose_temp2, tfpose_temp3);
				translateGripperY(0.37, tfpose_temp3, tfpose_homed);
				robot.move_to_point(tfpose_homed, "base_link", 0.1, client);
			}
			init_state = false;
			tool_offset.setValue(xoffset, yoffset, zoffset);
			ROS_INFO("READY TO MOVE");
		}



		input = getchar();
		if (input == 'c') {
			if (calibNum < calibSteps) {
				translateGripperX(calibX[calibNum], tfpose_homed, tfpose_temp1);
				translateGripperY(calibY[calibNum], tfpose_temp1, tfpose_temp2);
				translateGripperZ(calibZ[calibNum], tfpose_temp2, tfpose_goal);
				robot.move_to_point(tfpose_goal, "base_link", 0.1, client);
				calibNum++;
				ROS_INFO("@calibration step %d", calibNum);
			} else {
				ROS_INFO("Done with calibration");
			}
		} else if (input == 'x') {
			if (calibNum < calibSteps) {
				translateGripperX(calibX[calibNum], tfpose_homed, tfpose_temp1);
				translateGripperY(calibY[calibNum], tfpose_temp1, tfpose_temp2);
				translateGripperZ(calibZ[calibNum], tfpose_temp2, tfpose_goal);
				robot.move_to_point(tfpose_goal, "base_link", 0.1, client);
				calibNum--;
				ROS_INFO("@calibration step %d", calibNum);
			} else {
				ROS_INFO("Done with calibration");
			}
		} else if (input == 'n') {
			if (stepNum < 40) {
				translateGripperZ(positionZ[stepNum], tfpose_homed, tfpose_temp1);
				translateGripperX(positionX[stepNum], tfpose_temp1, tfpose_goal);
				robot.move_to_point(tfpose_goal, "base_link", 0.1, client);
				ROS_INFO("@data step %d", calibNum);
				stepNum++;
			} else {
				ROS_INFO("Done with data");
			}
		} else if (input == 'h') {
			stepNum = 0;
			calibNum = 0;
			robot.move_to_point(tfpose_homed, "base_link", 0.1, client);
			ROS_INFO("robot @ home");
		}
////	} else if (input == 'w') {
////		output_file << fixed << setprecision(5) << "hi" << "," << "hi" << endl;
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
////		}

        //ros::spinOnce();
    }

}


#endif
