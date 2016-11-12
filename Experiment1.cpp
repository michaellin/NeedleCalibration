#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <boost/thread/thread.hpp>
#include <robotiq_c_model_control/CModel_robot_input.h>
#include <robotiq_c_model_control/CModel_robot_output.h>
#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "tactilesensors4/StaticData.h"
#include "tactilesensors4/Accelerometer.h"
#include "tactilesensors4/Dynamic.h"
#include "tactilesensors4/Gyroscope.h"
#include "tactilesensors4/EulerAngle.h"
#include "sensor_msgs/JointState.h"
#include "roboticArm.h"
#include "URcontrolV2.h"

//Using boost and std namespaces
using namespace std;
using namespace boost::posix_time;

#define NUM_OF_TAXELS 28
#define XMOTION_INCREMENT 0.001
#define STATIC_SUM_TRESHOLD 200
#define NUM_OF_FTSENSOR_INITIALIZATION_ITER 500 // @ 100Hz, so this should last for about 5 seconds
#define NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER 1000 // @ 1kHz, this should last for a sec (remember though refresh rate is around 60Hz, so it's not 1000 data!).

//// Gripper global variables:
bool GripperIsAlive=false,FTSensorIsAlive=false,FTSensorNeedsToBeInitialized=false, TactileSensorsNeedToBeInitialized = false, StaticDataReadingRequested=false;
int NumOfFTSensorInitializationIterations=0,NumOfTactileSensorsInitializationIterations=0,TactileSensor1Sum=0,TactileSensor2Sum=0;

//Publisher de la pince
ros::Publisher gripper_publisher;
robotiq_c_model_control::CModel_robot_output gripper_command;

//Functions' Prototype (not callback functions):
bool cmdOptionExists(char** begin, char** end, const string& option);
char* getCmdOption(char ** begin, char ** end, const string & option);
std::string GetDataOutputFileName(int DesiredGripperForce, int DesiredLevelOfAdhesion, int DesiredTypeOfMotion, double DesiredSpeed);
void PerformGripperPatch();
bool VerifyGripperIsAlive();
void PerformFTSensorPatch();
bool VerifyFTSensorIsAlive();
void InitializeFTSensor();
void InitializeTactileSensors();
void RobotMotionThread(RoboticArm* robot,tf::Transform* APoseToReach1, int TypeOfMotion, double ToolOffset, double DesiredSpeed);
bool AreArgumentsValid(int DesiredGripperForce, int DesiredLevelOfAdhesion, int DesiredTypeOfMotion, double DesiredSpeed);

//Some Global Variables
bool NewTactileSensorDataHasArrived=false,RobotIsDonePullingThePart=false;
double Fx=0,Fy=0,Fz=0,Mx=0,My=0,Mz=0;
double FxBias=0,FyBias=0,FzBias=0,MxBias=0,MyBias=0,MzBias=0;
std::vector<double> TactileSensor1(28),TactileSensor2(28);
std::vector<double> TactileSensor1Bias(28),TactileSensor2Bias(28);
std::vector<double> TactileSensor1RealValues(28),TactileSensor2RealValues(28);
int16_t Dyn1=0, Dyn2=0, Ax1=0, Ay1=0, Az1=0, Ax2=0, Ay2=0, Az2=0, Gx1=0, Gy1=0, Gz1=0, Gx2=0, Gy2=0, Gz2=0;
uint8_t gACT=0, gCU=0, gFLT=0, gGTO=0, gPO=0, gSTA=0, gOBJ=0, gPR=0;
double Roll1=0, Pitch1=0, Yaw1=0, Roll2=0, Pitch2=0, Yaw2=0;




void staticdataCallback(const tactilesensors4::StaticData::ConstPtr &msg)
{

    if (TactileSensorsNeedToBeInitialized && NumOfTactileSensorsInitializationIterations<NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER)
    {
        for (int i=0;i<msg->taxels.data()->values.size();i++)
        {
            TactileSensor1.at(i)=TactileSensor1.at(i)+msg->taxels[0].values[i];
            TactileSensor2.at(i)=TactileSensor2.at(i)+msg->taxels[1].values[i];
        }
        NumOfTactileSensorsInitializationIterations++;
        std::cout << "NumOfTactileSensorsInitializations = " << NumOfTactileSensorsInitializationIterations << std::endl;
    }
    else if (NumOfTactileSensorsInitializationIterations==NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER)
    {
        for (int i=0;i<msg->taxels.data()->values.size();i++)
        {
            TactileSensor1Bias.at(i)=TactileSensor1.at(i)/NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER;
            TactileSensor2Bias.at(i)=TactileSensor2.at(i)/NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER;
            std::cout << "TactileSensor1Bias: " << TactileSensor1Bias.at(i) << std::endl;
        }
        TactileSensorsNeedToBeInitialized=false;
        NumOfTactileSensorsInitializationIterations++; // Just so we show this message only once
    }
    else
    {
        //        std::cout << "We finally reached here..." << std::endl;
        for (int i=0;i<msg->taxels.data()->values.size();i++)
        {
            TactileSensor1RealValues.at(i)=(int)(msg->taxels[0].values[i]-TactileSensor1Bias.at(i));
            TactileSensor2RealValues.at(i)=(int)(msg->taxels[1].values[i]-TactileSensor2Bias.at(i));
        }

        if (StaticDataReadingRequested)
        {
            std::cout << "Sum of the taxels located in the middle of sensor #1: " << TactileSensor1RealValues.at(9)+TactileSensor1RealValues.at(10)+TactileSensor1RealValues.at(13)+TactileSensor1RealValues.at(14)+TactileSensor1RealValues.at(17)+TactileSensor1RealValues.at(18) << std::endl;
            std::cout << "Sum of the taxels located in the middle of sensor #2: " << TactileSensor2RealValues.at(9)+TactileSensor2RealValues.at(10)+TactileSensor2RealValues.at(13)+TactileSensor2RealValues.at(14)+TactileSensor2RealValues.at(17)+TactileSensor2RealValues.at(18) << std::endl;
            StaticDataReadingRequested=false;
        }
        NewTactileSensorDataHasArrived=true;
    }
}

void dynamicCallback(const tactilesensors4::Dynamic::ConstPtr &msg)
{
    Dyn1=msg->data[0].value; Dyn2=msg->data[0].value;
}

void accelerometerCallback(const tactilesensors4::Accelerometer::ConstPtr &msg)
{
    Ax1=msg->data[0].values[0];Ay1=msg->data[0].values[1];Az1=msg->data[0].values[2];
    Ax2=msg->data[1].values[0];Ay2=msg->data[1].values[1];Az2=msg->data[1].values[2];
    //        ROS_INFO("We have received accelerometer data from sensor #1: ax=[%i], ay=[%i] and az=[%i]", msg->data[0].values[0], msg->data[0].values[1], msg->data[0].values[2]);
    //        ROS_INFO("We have received accelerometer data from sensor #2: ax=[%i], ay=[%i] and az=[%i]", msg->data[1].values[0], msg->data[1].values[1], msg->data[1].values[2]);
}

void gyroscopeCallback(const tactilesensors4::Gyroscope::ConstPtr &msg)
{
    Gx1=msg->data[0].values[0];Gy1=msg->data[0].values[1];Gz1=msg->data[0].values[2];
    Gx2=msg->data[1].values[0];Gy2=msg->data[1].values[1];Gz2=msg->data[1].values[2];
}

void eulerangleCallback(const tactilesensors4::EulerAngle::ConstPtr &msg)
{
    Roll1=msg->data[0].values[0]; Pitch1=msg->data[0].values[1]; Yaw1=msg->data[0].values[2];
    Roll2=msg->data[1].values[0]; Pitch2=msg->data[1].values[1]; Yaw2=msg->data[1].values[2];
}

void gripper_input_callback(const robotiq_c_model_control::CModel_robot_input::ConstPtr &msg)
{
    GripperIsAlive=true;
    //    ROS_INFO("Here's a message from the gripper: [%i]",msg->gACT);
    gACT=msg->gACT; gCU=msg->gCU; gFLT=msg->gFLT; gGTO=msg->gGTO; gOBJ=msg->gOBJ; gPO=msg->gPO; gPR=msg->gPR; gSTA=msg->gSTA;

}

void forcemomentsensorCallback(const robotiq_force_torque_sensor::ft_sensor::ConstPtr &msg)
{
    FTSensorIsAlive=true;
    if (FTSensorNeedsToBeInitialized && NumOfFTSensorInitializationIterations<NUM_OF_FTSENSOR_INITIALIZATION_ITER)
    {
        Fx=Fx+msg->Fx;Fy=Fy+msg->Fy;Fz=Fz+msg->Fz;
        Mx=Mx+msg->Mx;My=My+msg->My;Mz=Mz+msg->Mz;
        NumOfFTSensorInitializationIterations++;
    }
    else if (NumOfFTSensorInitializationIterations==NUM_OF_FTSENSOR_INITIALIZATION_ITER)
    {
        //Commented out for now since we will process raw data to preserve all info and treat them afterward...
        //        FxBias=Fx/NUM_OF_FTSENSOR_INITIALIZATION_ITER; FyBias=Fy/NUM_OF_FTSENSOR_INITIALIZATION_ITER; FzBias=Fz/NUM_OF_FTSENSOR_INITIALIZATION_ITER;
        //        MxBias=Mx/NUM_OF_FTSENSOR_INITIALIZATION_ITER; MyBias=My/NUM_OF_FTSENSOR_INITIALIZATION_ITER; MzBias=Mz/NUM_OF_FTSENSOR_INITIALIZATION_ITER;
        FTSensorNeedsToBeInitialized=false;
        NumOfFTSensorInitializationIterations++; // Just so we show this message only once
    }
    else
    {
        Fx=msg->Fx-FxBias;Fy=msg->Fy-FyBias;Fz=msg->Fz-FzBias;
        Mx=msg->Mx-MxBias;My=msg->My-MyBias;Mz=msg->Mz-MzBias;
    }
    //    ROS_INFO("Force and moment sensor: Fx=[%f], Fy=[%f], Fz=[%f], Mx=[%f], My=[%f] and Mz=[%f]",msg->Fx,msg->Fy,msg->Fz,msg->Mx,msg->My,msg->Mz);
}

void jointstatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{

}



// TO DO: to consider using an asynchronous spinner to loop in the while(ros::ok()) loop.
int main(int argc, char **argv)
{
    /******************* VARIABLES INITIALISATIONS ******************/
    int i = 0;
    double TaxelsSum=0,AdhesionSurfaceDepth=0;
    bool DetectOjbectCenterLine = false;
    ros::init(argc, argv, "experiment_1");
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    ros::Subscriber sub_static = n->subscribe("/TactileSensor4/StaticData", 1000, staticdataCallback);
    ros::Subscriber sub_dynamic = n->subscribe("/TactileSensor4/Dynamic", 1000,dynamicCallback);
    ros::Subscriber sub_accelerometer = n->subscribe("/TactileSensor4/Accelerometer",1000,accelerometerCallback);
    ros::Subscriber sub_gyroscopes = n->subscribe("/TactileSensor4/Gyroscope",1000,gyroscopeCallback);
    ros::Subscriber sub_euler_angle = n->subscribe("/TactileSensor4/EulerAngle",1000,eulerangleCallback);
    ros::Subscriber sub_gripper_input = n->subscribe("/CModelRobotInput", 1, gripper_input_callback);
    ros::Subscriber sub_force_moment_sensor = n->subscribe("/robotiq_force_torque_sensor",1000,forcemomentsensorCallback);
    ros::Subscriber sub_joint_statse = n->subscribe("/joint_states",1000,jointstatesCallback);
    ros::Rate loop_rate(1000);

    gripper_publisher = n->advertise<robotiq_c_model_control::CModel_robot_output> ("/CModelRobotOutput", 1);
    RoboticArm robot; //Robot control object
    std::vector<tf::Transform> ATrajectory; //Array of poses
    tf::Transform tfpose_init, tfpose_oriented, APoseToReach1; //Poses
    tf::Quaternion AQuaternionToReach1;
    tf::Vector3 ToolOffset, OriginalSetOfCoordinates, ASetOfCoordinnates, XPosLimit, XNegLimit, InitialPosition; //Tool offset
    ToolOffset.setValue(0.0,0.0,0.195); // This is somehow accurate, I measured it...
    int CurrentGripperClosure=0, DesiredGripperForce=-1, DesiredLevelOfAdhesion=-1, DesiredMotionType=-1;
    double DesiredSpeed=-1;
    int closure=0;

    //    boost::thread thread_b(RobotMotionThread, 1);

    if(cmdOptionExists(argv, argv+argc, "-INIT_OBJ_POS"))
        DetectOjbectCenterLine=true;

    if(cmdOptionExists(argv, argv+argc, "-FORCE"))
    {
        char * filename = getCmdOption(argv, argv + argc, "-FORCE");
        if (filename)
        {
            DesiredGripperForce=atoi(filename);
        }
    }

    if(cmdOptionExists(argv, argv+argc, "-ADHESION_LEVEL"))
    {
        char * filename = getCmdOption(argv, argv + argc, "-ADHESION_LEVEL");
        if (filename)
        {
            DesiredLevelOfAdhesion=atoi(filename);
        }
    }

    if(cmdOptionExists(argv, argv+argc, "-MOTION_TYPE"))
    {
        char * filename = getCmdOption(argv, argv + argc, "-MOTION_TYPE");
        if (filename)
        {
            DesiredMotionType=atoi(filename);
        }
    }
    if(cmdOptionExists(argv, argv+argc, "-SPEED"))
    {
        char * filename = getCmdOption(argv, argv + argc, "-SPEED");
        if (filename)
        {
            DesiredSpeed=atof(filename);
        }
    }

    if(!AreArgumentsValid(DesiredGripperForce,DesiredLevelOfAdhesion,DesiredMotionType,DesiredSpeed))
    {
        std::cout << "ERROR: Arguments are not valid, will now exit..." << std::endl;
        return 0;
    }


    closure=185+3*DesiredGripperForce;
    AdhesionSurfaceDepth=0.005*DesiredLevelOfAdhesion;

    /********************** FT SENSOR SETUP ************************/
    PerformFTSensorPatch();
    std::cout << "FTSensor Initialization will now proceed, wait for 5 seconds." << std::endl;
    InitializeFTSensor(); // Commented for now, I think it's better to get the raw data
    std::cout << "FTSensor Initialization completed. Biases are: " << FxBias << " " << FyBias << " " << FzBias << " " << MxBias << " " << MyBias << " " << MzBias << std::endl;

    /******************** TACTILE SENSORS SETUP *******************/
    std::cout << "Tactile Sensors Initialization will now proceed, please allow 1-2 seconds to complete." << std::endl;
    InitializeTactileSensors();
    std::cout << "Tactile Sensors Initialization completed." << std::endl;

    /************* INITIAL POSITIONS AND ORIENTATIONS **************/
    tfpose_init = robot.get_arm_position("base_link");
    orientEffectorTowardsTable(tfpose_init, tfpose_oriented); //Calculate matrix to orient gripper towards table
    ASetOfCoordinnates.setValue(0,0.4,0.4); //Home Position
    APoseToReach1.setBasis(tfpose_oriented.getBasis());
    APoseToReach1.setOrigin(ASetOfCoordinnates);
    ROS_INFO("READY TO MOVE");
    robot.move_to_point(APoseToReach1,"base_link", 0.1); //Move to only one position

    /*********************** GRASPING SETUP ************************/
    PerformGripperPatch();
    std::cout << "Wait for a few seconds" << std::endl;
    ros::Duration(3).sleep();

    /************* APPROACH POSITION AND ORIENTATION **************/
    ASetOfCoordinnates.setValue(0.00844621,0.566545,0.440464); //Approach Position 1
    AQuaternionToReach1.setX(0.999783); AQuaternionToReach1.setY(0.0202454); AQuaternionToReach1.setZ(-0.00486366); AQuaternionToReach1.setW(0.000476199);
    APoseToReach1.setRotation(AQuaternionToReach1);
    APoseToReach1.setOrigin(ASetOfCoordinnates);
    ROS_INFO("READY TO MOVE, PRESS ENTER TO START THE EXPERIMENT."); getchar();
    robot.move_to_point(APoseToReach1,"base_link",0.1); //Move to only one position

    ASetOfCoordinnates.setValue(0.00844621,0.566545,0.340464); //Approach Position 2
    APoseToReach1.setOrigin(ASetOfCoordinnates);
    ROS_INFO("READY TO MOVE");
    robot.move_to_point(APoseToReach1,"base_link",0.1); //Move to only one position

    /************ PROCEDURE TO REACH CENTER LINE ****************/
    if(DetectOjbectCenterLine)
    {
        std::ofstream OutputFile("/home/bdml/catkin_ws/ExperimentalData/CenterPos.txt");

        // PALPING TO FIND GRASP X CENTER POSITION
        APoseToReach1 = robot.get_arm_position("base_link"); ASetOfCoordinnates=APoseToReach1.getOrigin(); InitialPosition=ASetOfCoordinnates;

        // Moving in +x direction and sensing contact:
        while (TaxelsSum<STATIC_SUM_TRESHOLD)
        {
            std::cout << "Moving toward x+" << std::endl;
            ASetOfCoordinnates.setX(ASetOfCoordinnates.getX()+XMOTION_INCREMENT);
            APoseToReach1.setOrigin(ASetOfCoordinnates);
            ros::spinOnce();
            TaxelsSum=TactileSensor1RealValues.at(9)+TactileSensor1RealValues.at(10)+TactileSensor1RealValues.at(13)+TactileSensor1RealValues.at(14)+TactileSensor1RealValues.at(17)+TactileSensor1RealValues.at(18);
            robot.move_to_point(APoseToReach1,"base_link",0.01); //Move at slow speed
            std::cout << "TaxelsSum = " << TaxelsSum << std::endl;
        }
        XPosLimit=robot.get_arm_position("base_link").getOrigin();
        ASetOfCoordinnates=InitialPosition;
        APoseToReach1.setOrigin(ASetOfCoordinnates);
        std::cout << "We will now move back to the initial position..." << std::endl;
        getchar();
        robot.move_to_point(APoseToReach1,"base_link",0.01); //Move back to center position

        TaxelsSum=0;
        //Now, moving in the -x direction
        while (TaxelsSum<STATIC_SUM_TRESHOLD)
        {
            std::cout << "Moving toward x-" << std::endl;
            ASetOfCoordinnates.setX(ASetOfCoordinnates.getX()-XMOTION_INCREMENT);
            APoseToReach1.setOrigin(ASetOfCoordinnates);
            ros::spinOnce();
            TaxelsSum=TactileSensor2RealValues.at(9)+TactileSensor2RealValues.at(10)+TactileSensor2RealValues.at(13)+TactileSensor2RealValues.at(14)+TactileSensor2RealValues.at(17)+TactileSensor2RealValues.at(18);
            robot.move_to_point(APoseToReach1,"base_link",0.01); //Move at slow speed
            std::cout << "TaxelsSum = " << TaxelsSum << std::endl;
        }
        XNegLimit=robot.get_arm_position("base_link").getOrigin();
        APoseToReach1.setOrigin(InitialPosition);
        robot.move_to_point(APoseToReach1,"base_link",0.01); //Move back to center position

        ASetOfCoordinnates.setValue(0.00844621,0.566545,0.440464);
        APoseToReach1.setOrigin(ASetOfCoordinnates);
        robot.move_to_point(APoseToReach1,"base_link",0.1); //Moving away... (TMP)

        std::cout << "XPosLimit = " << XPosLimit.getX() << " XNegLimit = " << XNegLimit.getX() << std::endl;
        //        std::cout << "Computed Center X Coordinate would then be: " << (XPosLimit.getX()-XNegLimit.getX())/2+XNegLimit.getX() << std::endl;
        OutputFile << (XPosLimit.getX()-XNegLimit.getX())/2+XNegLimit.getX(); // Write the information in the text file
        OutputFile.close();
    }
    else
    {
        // This is just to make sure everything is ok...
        StaticDataReadingRequested=true; ros::spinOnce();
        // Then we read the position indicated in the file
        string CenteredXPos;
        ifstream infile;
        infile.open("/home/bdml/catkin_ws/ExperimentalData/CenterPos.txt");

        getline(infile,CenteredXPos); // Saves the line in CenteredXPos.
        //        std::cout << "This is the content of the CenteredXPos : " << atof(CenteredXPos.c_str()) << std::endl;
        infile.close();

        ASetOfCoordinnates.setX(atof(CenteredXPos.c_str()));
        APoseToReach1.setOrigin(ASetOfCoordinnates);
        std::cout << "Will now re-adjust the X coordinate according to the part's center line. Type ENTER to proceed." << std::endl;
        robot.move_to_point(APoseToReach1,"base_link",0.1);
        std::cout << "Done!" << std::endl;
    }

    /**************** MOVING TO THE START POINT ***********************/
    ASetOfCoordinnates=robot.get_arm_position("base_link").getOrigin();
    ASetOfCoordinnates.setZ(ASetOfCoordinnates.getZ()+0.0425);
    APoseToReach1.setOrigin(ASetOfCoordinnates);
    robot.move_to_point(APoseToReach1,"base_link",0.1);


    /**************** GETTING A FIRST SNAPSHOT *******************/
    // Time measurement variables:
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    std::ofstream DataOutputFile(GetDataOutputFileName(DesiredGripperForce,DesiredLevelOfAdhesion,DesiredMotionType,DesiredSpeed).c_str());
    struct timeval TimeStart, TimeEnd;
    long mtime, seconds, useconds;

    ros::Duration(1).sleep();
    std::cout << "An initial data snapshot will now be taken, wait for one second." << std::endl;
    // First Data snapshot:
    gettimeofday(&TimeStart, NULL);
    DataOutputFile << "Data acquired on " << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' <<  now->tm_mday << ", Initial Snapshot:," << std::endl;
    int TactileSensorDataCounter=1; NewTactileSensorDataHasArrived=false;
    while (TactileSensorDataCounter<1000)
    {
        ros::spinOnce();
        if(NewTactileSensorDataHasArrived)
        {
            gettimeofday(&TimeEnd, NULL);
            seconds  = TimeEnd.tv_sec  - TimeStart.tv_sec;
            useconds = TimeEnd.tv_usec - TimeStart.tv_usec;
            mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

            DataOutputFile << mtime << "," << TactileSensorDataCounter << ",FT," << Fx << "," << Fy << "," << Fz << "," << Mx << "," << My << "," << Mz << ",";
            // Writing this all manually instead of relying on a loop or function to (try to) save execution a little:
            DataOutputFile << "S1," << TactileSensor1RealValues.at(0) << "," << TactileSensor1RealValues.at(1) << "," << TactileSensor1RealValues.at(2) << "," << TactileSensor1RealValues.at(3) << "," << TactileSensor1RealValues.at(4) << "," << TactileSensor1RealValues.at(5) << "," << TactileSensor1RealValues.at(6) << "," << +
                              TactileSensor1RealValues.at(7) << "," << TactileSensor1RealValues.at(8) << "," << TactileSensor1RealValues.at(9) << "," << TactileSensor1RealValues.at(10) << "," << TactileSensor1RealValues.at(11) << "," << TactileSensor1RealValues.at(12) << "," << TactileSensor1RealValues.at(13) << "," << +
                              TactileSensor1RealValues.at(14) << "," << TactileSensor1RealValues.at(15) << "," << TactileSensor1RealValues.at(16) << "," << TactileSensor1RealValues.at(17) << "," << TactileSensor1RealValues.at(18) << "," << TactileSensor1RealValues.at(19) << "," << TactileSensor1RealValues.at(20) << "," << +
                              TactileSensor1RealValues.at(21) << "," << TactileSensor1RealValues.at(22) << "," << TactileSensor1RealValues.at(23) << "," << TactileSensor1RealValues.at(24) << "," << TactileSensor1RealValues.at(25) << "," << TactileSensor1RealValues.at(26) << "," << TactileSensor1RealValues.at(27) << "," ;
            DataOutputFile << "S2," << TactileSensor2RealValues.at(0) << "," << TactileSensor2RealValues.at(1) << "," << TactileSensor2RealValues.at(2) << "," << TactileSensor2RealValues.at(3) << "," << TactileSensor2RealValues.at(4) << "," << TactileSensor2RealValues.at(5) << "," << TactileSensor2RealValues.at(6) << "," << +
                              TactileSensor2RealValues.at(7) << "," << TactileSensor2RealValues.at(8) << "," << TactileSensor2RealValues.at(9) << "," << TactileSensor2RealValues.at(10) << "," << TactileSensor2RealValues.at(11) << "," << TactileSensor2RealValues.at(12) << "," << TactileSensor2RealValues.at(13) << "," << +
                              TactileSensor2RealValues.at(14) << "," << TactileSensor2RealValues.at(15) << "," << TactileSensor2RealValues.at(16) << "," << TactileSensor2RealValues.at(17) << "," << TactileSensor2RealValues.at(18) << "," << TactileSensor2RealValues.at(19) << "," << TactileSensor2RealValues.at(20) << "," << +
                              TactileSensor2RealValues.at(21) << "," << TactileSensor2RealValues.at(22) << "," << TactileSensor2RealValues.at(23) << "," << TactileSensor2RealValues.at(24) << "," << TactileSensor2RealValues.at(25) << "," << TactileSensor2RealValues.at(26) << "," << TactileSensor2RealValues.at(27) << "," ;
            DataOutputFile << "D1," << Dyn1 << "," << "D2," << Dyn2 << ",";
            DataOutputFile << "Acc1," << Ax1 << "," << Ay1 << "," << Az1 << "," << "Acc2," << Ax2 << "," << Ay2 << "," << Az2 << ",";
            DataOutputFile << "Gyr1," << Gx1 << "," << Gy1 << "," << Gz1 << "," << "Gyr2," << Gx2 << "," << Gy2 << "," << Gz2 << ",";
            DataOutputFile << "EA1," << Roll1 << "," << Pitch1 << "," << Yaw1 << ",EA2," << Roll2 << "," << Pitch2 << "," << Yaw2 << ",";
            DataOutputFile << "GRIP," << (int)gACT << "," << (int)gCU << "," << (int)gFLT << "," << (int)gGTO << "," << (int)gPO << "," << (int)gSTA << "," << (int)gOBJ << "," << (int)gPR << ",";
            DataOutputFile << "ROBOTSP," << APoseToReach1.getOrigin().getX() << "," << APoseToReach1.getOrigin().getY() << "," << APoseToReach1.getOrigin().getZ() << "," << +
                              APoseToReach1.getRotation().getX() << "," << APoseToReach1.getRotation().getY() << "," << APoseToReach1.getRotation().getZ() << "," << std::endl;
            TactileSensorDataCounter++;
            NewTactileSensorDataHasArrived=false;
        }
    }
    std::cout << "Done!" << std::endl;

    OriginalSetOfCoordinates=robot.get_arm_position("base_link").getOrigin(); // This will give the initial reference

    // 215 - 230 (remeber that range for experiments later)
    //    for (closure=185; closure<=200; closure=closure+3)
    //    {



    //    for (double AdhesionSurfaceDepth=0;AdhesionSurfaceDepth<0.035;AdhesionSurfaceDepth=AdhesionSurfaceDepth+0.005)
    //    {


    // Moving to the current depth, changing adhesion surface
    std::cout << "This is the new coordinate we will go to : " << ASetOfCoordinnates.getZ()+AdhesionSurfaceDepth << " and the surface depth is now: " << AdhesionSurfaceDepth << std::endl;
    ASetOfCoordinnates.setZ(OriginalSetOfCoordinates.getZ()-AdhesionSurfaceDepth);
    APoseToReach1.setOrigin(ASetOfCoordinnates);
    robot.move_to_point(APoseToReach1,"base_link",0.1);

    // That's to progressively approach the desired gripper closure from a safe starting point:
    CurrentGripperClosure=180;
    while (CurrentGripperClosure<=closure)
    {
        gripper_command.rACT = 1;
        gripper_command.rSP=0;
        gripper_command.rFR=0;
        gripper_command.rPR=CurrentGripperClosure;
        gripper_command.rGTO = 1;
        gripper_publisher.publish(gripper_command);
        ros::Duration(1).sleep();
        CurrentGripperClosure++;
    }


    RobotIsDonePullingThePart=false;
    boost::thread thread_a(RobotMotionThread, &robot, &APoseToReach1, DesiredMotionType,0.195, DesiredSpeed);
    while (!RobotIsDonePullingThePart && ros::ok())
    {
        ros::spinOnce();
        gettimeofday(&TimeEnd, NULL);
        seconds  = TimeEnd.tv_sec  - TimeStart.tv_sec;
        useconds = TimeEnd.tv_usec - TimeStart.tv_usec;
        mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

        DataOutputFile << mtime << "," << TactileSensorDataCounter << ",FT," << Fx << "," << Fy << "," << Fz << "," << Mx << "," << My << "," << Mz << ",";
        // Writing this all manually instead of relying on a loop or function to (try to) save execution a little:
        DataOutputFile << "S1," << TactileSensor1RealValues.at(0) << "," << TactileSensor1RealValues.at(1) << "," << TactileSensor1RealValues.at(2) << "," << TactileSensor1RealValues.at(3) << "," << TactileSensor1RealValues.at(4) << "," << TactileSensor1RealValues.at(5) << "," << TactileSensor1RealValues.at(6) << "," << +
                          TactileSensor1RealValues.at(7) << "," << TactileSensor1RealValues.at(8) << "," << TactileSensor1RealValues.at(9) << "," << TactileSensor1RealValues.at(10) << "," << TactileSensor1RealValues.at(11) << "," << TactileSensor1RealValues.at(12) << "," << TactileSensor1RealValues.at(13) << "," << +
                          TactileSensor1RealValues.at(14) << "," << TactileSensor1RealValues.at(15) << "," << TactileSensor1RealValues.at(16) << "," << TactileSensor1RealValues.at(17) << "," << TactileSensor1RealValues.at(18) << "," << TactileSensor1RealValues.at(19) << "," << TactileSensor1RealValues.at(20) << "," << +
                          TactileSensor1RealValues.at(21) << "," << TactileSensor1RealValues.at(22) << "," << TactileSensor1RealValues.at(23) << "," << TactileSensor1RealValues.at(24) << "," << TactileSensor1RealValues.at(25) << "," << TactileSensor1RealValues.at(26) << "," << TactileSensor1RealValues.at(27) << "," ;
        DataOutputFile << "S2," << TactileSensor2RealValues.at(0) << "," << TactileSensor2RealValues.at(1) << "," << TactileSensor2RealValues.at(2) << "," << TactileSensor2RealValues.at(3) << "," << TactileSensor2RealValues.at(4) << "," << TactileSensor2RealValues.at(5) << "," << TactileSensor2RealValues.at(6) << "," << +
                          TactileSensor2RealValues.at(7) << "," << TactileSensor2RealValues.at(8) << "," << TactileSensor2RealValues.at(9) << "," << TactileSensor2RealValues.at(10) << "," << TactileSensor2RealValues.at(11) << "," << TactileSensor2RealValues.at(12) << "," << TactileSensor2RealValues.at(13) << "," << +
                          TactileSensor2RealValues.at(14) << "," << TactileSensor2RealValues.at(15) << "," << TactileSensor2RealValues.at(16) << "," << TactileSensor2RealValues.at(17) << "," << TactileSensor2RealValues.at(18) << "," << TactileSensor2RealValues.at(19) << "," << TactileSensor2RealValues.at(20) << "," << +
                          TactileSensor2RealValues.at(21) << "," << TactileSensor2RealValues.at(22) << "," << TactileSensor2RealValues.at(23) << "," << TactileSensor2RealValues.at(24) << "," << TactileSensor2RealValues.at(25) << "," << TactileSensor2RealValues.at(26) << "," << TactileSensor2RealValues.at(27) << "," ;
        DataOutputFile << "D1," << Dyn1 << "," << "D2," << Dyn2 << ",";
        DataOutputFile << "Acc1," << Ax1 << "," << Ay1 << "," << Az1 << "," << "Acc2," << Ax2 << "," << Ay2 << "," << Az2 << ",";
        DataOutputFile << "Gyr1," << Gx1 << "," << Gy1 << "," << Gz1 << "," << "Gyr2," << Gx2 << "," << Gy2 << "," << Gz2 << ",";
        DataOutputFile << "EA1," << Roll1 << "," << Pitch1 << "," << Yaw1 << ",EA2," << Roll2 << "," << Pitch2 << "," << Yaw2 << ",";
        DataOutputFile << "GRIP," << (int)gACT << "," << (int)gCU << "," << (int)gFLT << "," << (int)gGTO << "," << (int)gPO << "," << (int)gSTA << "," << (int)gOBJ << "," << (int)gPR << ",";
        DataOutputFile << "ROBOTSP," << APoseToReach1.getOrigin().getX() << "," << APoseToReach1.getOrigin().getY() << "," << APoseToReach1.getOrigin().getZ() << "," << +
                          APoseToReach1.getRotation().getX() << "," << APoseToReach1.getRotation().getY() << "," << APoseToReach1.getRotation().getZ() << ",";
        DataOutputFile << std::endl << std::endl;
        // We definitely stop the robot motion thread, if it was not already naturally stopped before:
        loop_rate.sleep();
        //        //move the robot upward and start logging everything
        //        //reopen the gripper
    }

    thread_a.interrupt();
    thread_a.join();
    // We reopen the gripper
    gripper_command.rACT = 1;
    gripper_command.rSP=0;
    gripper_command.rFR=0;
    gripper_command.rPR=0;
    gripper_command.rGTO = 1;
    gripper_publisher.publish(gripper_command);
    ros::Duration(1).sleep();
    //    }
    //    }

    DataOutputFile.close();


    // --> Put an initial all-sensor reading in the data file before going further in the process.

    // These experiments might be invasive and may damage the sensors. That could be the last thing to test since it might damage permanently the gecko adhesive and / or the tactile sensors.
    // Before doing this, we should make sure that at least, FT sensor's z axis is colinear with UR's z axis, such that no parasite moments will be created when we'll pull the part from Z-axis. (if there's actually a way to that carefully)
    // For each FN=normal force, AS=adhesion surface
    // Move robot so AS is as desired (we should begin at low AS)
    // Close gripper according to desired FN (@ slow speed) (We should begin at low FN)
    //      Acquire Tactile sensors and FT at least while moving the robot upward (tangential force experiment)
    //      Acquire Tactile sensors and FT at least while rotating the robot (Moment experiment)
    // --> How can we then detect when we reach the failing point (slippage)? Visually? IMUs? Static Data (reduction of AS, e.g.)

    // Is there any other experiments that we could do before going there ^



    //Fill the array
    //    ATrajectory.push_back(tfpose_corner1);

    //    ROS_INFO("READY TO MOVE");
    //    getchar();
    //    robot.execute_trajectory(ATrajectory, 0.1, "base_link");

    //Main loop:
    //    while(ros::ok())
    //    {
    //        //        std::cout<< "We are in the loop" << std::endl;
    //        ros::spinOnce();
    //        //        if (RobotIsDonePullingThePart)
    //        //            std::cout << "Robot is done pulling the part!" << std::endl;
    //    }
    ROS_INFO("Program termination.");
    ros::shutdown();
    std::cout << "Done!" << std::endl;
    return 0;
}


bool VerifyGripperIsAlive()
{
    GripperIsAlive=false; // Remember, this is a global variable
    ros::Duration(1).sleep(); // If the gripper is alive, "GripperIsAlive" will become true here
    ros::spinOnce();
    return GripperIsAlive;
}

bool VerifyFTSensorIsAlive()
{
    FTSensorIsAlive=false; // Remember, this is a global variable
    ros::Duration(1).sleep(); // If the gripper is alive, "GripperIsAlive" will become true here
    ros::spinOnce();
    return FTSensorIsAlive;
}


void PerformGripperPatch()
{
    int CMDStatus;
    while (!VerifyGripperIsAlive())
    {
        std::cout << "The gripper has died! Re_initialization in progress. Please wait." << std::endl;
        CMDStatus = system("echo flylikeagecko | sudo -b -S chmod 777 /dev/ttyUSB* > /dev/null");
        CMDStatus = system("rosrun robotiq_c_model_control CModelRtuNode.py /dev/ttyUSB1 > /dev/null &");
        if (CMDStatus!=0)
        {
            CMDStatus = system("rosrun robotiq_c_model_control CModelRtuNode.py /dev/ttyUSB0 > /dev/null &");
        }
        ros::Duration(1).sleep();
    }
    gripper_command.rACT = 0;
    gripper_command.rGTO = 0;
    gripper_publisher.publish(gripper_command);
    ros::Duration(0.3).sleep();
    gripper_command.rACT = 1;
    gripper_command.rGTO =0;
    gripper_publisher.publish(gripper_command);
    ros::Duration(0.3).sleep();
    gripper_command.rACT = 1;
    gripper_command.rSP=0;
    gripper_command.rFR=0;
    gripper_command.rPR=0;
    gripper_command.rGTO = 1;
    gripper_publisher.publish(gripper_command);
    ros::Duration(2).sleep();
    std::cout << "The gripper is alive!" << std::endl;
}

void PerformFTSensorPatch()
{
    int CMDStatus;
    while (!VerifyFTSensorIsAlive())
    {
        std::cout << "The FTSensor is not working! Re_initialization in progress. Please wait..." << std::endl;
        CMDStatus = system("echo flylikeagecko | sudo -b -S chmod 777 /dev/ttyUSB* > /dev/null");
        CMDStatus = system("rosrun robotiq_force_torque_sensor rq_sensor > /dev/null &");
        ros::Duration(1).sleep();
    }
    std::cout << "The FTSensor is alive!" << std::endl;
}

void InitializeFTSensor()
{
    FTSensorNeedsToBeInitialized=true;
    while(FTSensorNeedsToBeInitialized)
    {
        ros::spinOnce();
    }
    std::cout << "Force and Torque Sensor has been initialized!" << std::endl;
}

void InitializeTactileSensors()
{
    TactileSensorsNeedToBeInitialized=true;
    while(TactileSensorsNeedToBeInitialized)
    {
        ros::spinOnce();
    }
    std::cout << "Tactile Sensors have been initialized!" << std::endl;
}


/*****************************************************************************************
//Function: cmdOptionExists
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns true, otherwise it will return false.
//
*****************************************************************************************/
bool cmdOptionExists(char** begin, char** end, const string& option)
{
    return find(begin, end, option) != end;
}

/****************************************************************************************
//Function: getCmdOption
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns a pointer pointing just after the string that was found.
//
****************************************************************************************/
char* getCmdOption(char ** begin, char ** end, const string & option)
{
    char ** itr = find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

std::string GetDataOutputFileName(int DesiredGripperForce, int DesiredLevelOfAdhesion, int DesiredTypeOfMotion, double DesiredSpeed)
{
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    string Indice;
    std::string OutputFileName;
    ifstream infile;
    infile.open("/home/bdml/catkin_ws/ExperimentalData/Indice.txt");
    getline(infile,Indice);
    infile.close();

    // Update Data Indice:
    std::ofstream OutputFile("/home/bdml/catkin_ws/ExperimentalData/Indice.txt");
    int TheIndice=(int)(atof(Indice.c_str()))+1;
    OutputFile << TheIndice;
    OutputFile.close();

    std::ostringstream stringStream;
    stringStream << "/home/bdml/catkin_ws/ExperimentalData/Data" << TheIndice << "_" << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' <<  now->tm_mday << +
                    "_F" << DesiredGripperForce << "_S" << DesiredSpeed << "_A" << DesiredLevelOfAdhesion << "_M" << DesiredTypeOfMotion << ".csv";

    return stringStream.str();

}


void RobotMotionThread(RoboticArm *robot, tf::Transform *APoseToReach1, int TypeOfMotion, double ToolOffsetZCoordinate, double DesiredSpeed)
{
    tf::Vector3 ToolOffset; //Tool offset
    ToolOffset.setValue(0.0,0.0,ToolOffsetZCoordinate); // This is somehow accurate, I measured it...
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

    std::vector<tf::Transform> trajectory;
    tf::Transform APoseToReach2,APoseToReach3;

    trajectory.clear();

    std::cout << "TypeOfMotion is set at " << TypeOfMotion << std::endl;
    if (TypeOfMotion==1) // Translational: To test Tangential force
    {
        trajectory.push_back(*(APoseToReach1));
        translateGripperZ(-0.055,*(APoseToReach1),APoseToReach2);
        trajectory.push_back(APoseToReach2);
    }
    else if (TypeOfMotion==2) //Rotational: To test Moment constraints
    {
        trajectory.push_back(*(APoseToReach1));
        rotateGripperX(M_PI/4,ToolOffset,*(APoseToReach1),APoseToReach2);
        trajectory.push_back(APoseToReach2);
    }
    else if(TypeOfMotion==3) // A combination of linear and rotational motions (limit surface model)
    {
        trajectory.push_back(*(APoseToReach1));
        translateGripperZ(-0.055,*(APoseToReach1),APoseToReach2);
        rotateGripperX(M_PI/5.95,ToolOffset,APoseToReach2,APoseToReach3); // This angle is just to have a nice rotational VS translational ratio
        trajectory.push_back(APoseToReach3);
    }
    //We start pulling the part at very low speed:
    robot->execute_trajectory(trajectory, 0.01, "base_link");
    RobotIsDonePullingThePart=true;

}

bool AreArgumentsValid(int DesiredGripperForce, int DesiredLevelOfAdhesion, int DesiredTypeOfMotion, double DesiredSpeed)
{
    if (DesiredGripperForce>=0 && DesiredGripperForce<=5 && DesiredLevelOfAdhesion>=0 && DesiredLevelOfAdhesion<=7 && DesiredTypeOfMotion>=1 && DesiredTypeOfMotion<=3 && DesiredSpeed>0 && DesiredSpeed<0.1)
    {
        return true;
    }
    return false;
}
