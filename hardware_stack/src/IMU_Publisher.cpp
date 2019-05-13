#include "ros/ros.h"
#include "message_files/imu_msgs.h"
#include <cstdlib>
#include <RTIMULib.h>
static const double G_TO_MPSS = 9.80665;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IMU_Publisher");
    ROS_INFO("Imu driver is now running");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<message_files::imu_msgs>("imu_data", 1000);
    //std::string calibration_file_path;
    //  Using RTIMULib here allows it to use the .ini file generated by RTIMULibDemo.
    //  Or, you can create the .ini in some other directory by using:
    //      RTIMUSettings *settings = new RTIMUSettings("<directory path>", "RTIMULib");
    //  where <directory path> is the path to where the .ini file is to be loaded/saved
    /*if(!nh.getParam("calibration_file_path", calibration_file_path))
    {
        ROS_ERROR("The calibration_file_path parameter must be set to use a "
                  "calibration file.");
        ROS_BREAK();
    }

    std::string calibration_file_name = "RTIMULib";
    if(!nh.getParam("calibration_file_name", calibration_file_name))
        ROS_WARN_STREAM("No calibration_file_name provided - default: "<< calibration_file_name);

    std::string frame_id = "imu_link";
    if(!nh.getParam("frame_id", frame_id))
        ROS_WARN_STREAM("No frame_id provided - default: " << frame_id);*/


    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    //  This is an opportunity to manually override any settings before the call IMUInit

    //  set up IMU

    imu->IMUInit();

    //  this is a convenient place to change fusion parameters

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    //  now just process data

    //  poll at the rate recommended by the IMU
    message_files::imu_msgs imu_msg;
    while (ros::ok())
    {

        if (imu->IMURead()) 
        {
            RTIMU_DATA imu_data = imu->getIMUData();
            ROS_INFO("getting data");
            //imu_msg.header.stamp = ros::Time::now();
            //imu_msg.header.frame_id = frame_id;

            imu_msg.roll = 100*imu_data.fusionPose.x();
            if(imu_data.fusionPose.x() > 0)
            {	imu_msg.roll = 100*imu_data.fusionPose.x();}
            else
            { imu_msg.roll = 360 + 100*imu_data.fusionPose.x();}
            //imu_msg.orientation.w = imu_data.fusionQPose.scalar();
            imu_msg.pitch = 100*imu_data.fusionPose.y();
            
            if(imu_data.fusionPose.y() > 0)
            {	imu_msg.pitch = 100*imu_data.fusionPose.y();}
            else
            { imu_msg.pitch = 360 + 100*imu_data.fusionPose.y();}
            //imu_msg.yaw = 100*imu_data.fusionPose.z();
            //if(imu_data.fusionPose.z() > 0)
            //{	imu_msg.yaw = 100*imu_data.fusionPose.z();}
            //else
            //{ imu_msg.yaw = 360 + 100*imu_data.fusionPose.z();}
            //imu_msg.orientation.w = imu_data.fusionQPose.scalar(); 

            imu_msg.angular_velocity_x = imu_data.gyro.x();
            imu_msg.angular_velocity_y = imu_data.gyro.y();
            imu_msg.angular_velocity_z = imu_data.gyro.z();

            imu_msg.linear_acceleration_x = imu_data.accel.x() * G_TO_MPSS;
            imu_msg.linear_acceleration_y = imu_data.accel.y() * G_TO_MPSS;
            imu_msg.linear_acceleration_z = imu_data.accel.z() * G_TO_MPSS;

            ROS_INFO("IMU data roll:%f , pitch:%f", imu_msg.roll, imu_msg.pitch);
            pub.publish(imu_msg);
        }
        ros::spinOnce();
        ros::Duration(imu->IMUGetPollInterval() / 1000.0).sleep();
    }
    return 0;
}
