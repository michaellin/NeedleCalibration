#include "ros/ros.h"
#include "roboticArm.h"
#include "URcontrolV2.h"
#include "tactilesensors4/StaticData.h"

#include <iostream>
#include <fstream>

void staticdataCallback(const tactilesensors4::StaticData::ConstPtr &msg)
{
//    for (int i=0;i<msg->taxels.data()->values.size();i++)
//    {
//        ROS_INFO("Taxels %i from Sensor #1 = [%i] and Sensor #2 = [%i]", i,msg->taxels[0].values[i],msg->taxels[1].values[i]);
//    }
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle n; /* necessaire pour que ros::ok retourne true ??? */
    ros::Subscriber sub_static = n.subscribe("/TactileSensor4/StaticData", 1000, staticdataCallback);

    //Robot control object
    RoboticArm robot;

    //Array of poses
    std::vector<tf::Transform> trajectory;

    //Poses
    tf::Transform tfpose_init, tfpose_oriented;
    tf::Transform tfpose_corner1, tfpose_corner2, tfpose_corner3, tfpose_corner4;


    while(ros::ok())
    {
        //Tool offset
        tf::Vector3 tool_offset;
        tool_offset.setValue(0.0,0.0,0.195);

        tfpose_init = robot.get_arm_position("base_link");

        //Calculate matrix to orient gripper towards table
        orientEffectorTowardsTable(tfpose_init, tfpose_oriented);


        ROS_INFO("READY TO MOVE");
        getchar();

        //Move to only one position
        robot.move_to_point(tfpose_oriented,"base_link",0.1);

        //Calculate 4 poses to make a square;
        translateGripperZ(-0.15,tfpose_oriented,tfpose_corner1);
        translateGripperX(0.15,tfpose_corner1,tfpose_corner2);
        translateGripperZ(0.15,tfpose_corner2,tfpose_corner3);
        translateGripperX(-0.15,tfpose_corner3,tfpose_corner4);

        //Clear trajectory array
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
