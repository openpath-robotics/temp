//
// Created by jh on 24. 11. 27.
//
#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"
#include <iostream>
#include <string>
#include <vector>
#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

#define PHOXI_PCL_SUPPORT
#define DEGREE2RAD 3.141592/180.0
#include "PhoXi.h"

///
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
//#include "point_types.h"
///

void convertToPCL(const pho::api::PFrame &Frame);
void convertToOctomap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &PCLCloud);
void matrixTransformation(const double *joints,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceCloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &transformedCloud);
void startSoftwareTriggerExample(pho::api::PPhoXi &PhoXiDevice);

double tempJointStates[6] ={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double resolution = 0.1; // OctoMap의 해상도
octomap::OcTree tree(resolution);

int main(int argc, char *argv[]) {
    pho::api::PhoXiFactory Factory;

    // Check if the PhoXi Control Software is running
    if (!Factory.isPhoXiControlRunning()) {
        std::cout << "PhoXi Control Software is not running" << std::endl;
        return 0;
    }

    // Get List of available devices on the network
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList =
        Factory.GetDeviceList();
    if (DeviceList.empty()) {
        std::cout << "PhoXi Factory has found 0 devices" << std::endl;
        return 0;
    }

    // Try to connect device opened in PhoXi Control, if any
    pho::api::PPhoXi PhoXiDevice = Factory.CreateAndConnectFirstAttached();
    if (PhoXiDevice) {
        std::cout << "You have already PhoXi device opened in PhoXi Control, "
                     "the API Example is connected to device: "
            << (std::string)PhoXiDevice->HardwareIdentification
            << std::endl;
    } else {
        std::cout
            << "You have no PhoXi device opened in PhoXi Control, the API ";
        for (size_t i = 0; i < DeviceList.size(); i++) {
            std::cout << "Example will try to connect to ..."
                << DeviceList.at(i).HWIdentification << std::endl;
            // wait 5 second for scanner became ready
            PhoXiDevice = Factory.CreateAndConnect(
                DeviceList.at(i).HWIdentification, 5000);
            if (PhoXiDevice) {
                std::cout << "succesfully connected" << std::endl;
                break;
            }
            if (i == DeviceList.size() - 1) {
                std::cout << "Can not connect to any device" << std::endl;
            }
        }
    }

    // Check if device was created
    if (!PhoXiDevice) {
        std::cout << "Your device was not created!" << std::endl;
        return 0;
    }

    // Check if device is connected
    if (!PhoXiDevice->isConnected()) {
        std::cout << "Your device is not connected" << std::endl;
        return 0;
    }

    // Launch software trigger example
    startSoftwareTriggerExample(PhoXiDevice);

    // Disconnect PhoXi device
    PhoXiDevice->Disconnect();
    return 0;
}

void convertToPCL(const pho::api::PFrame &Frame) {
    std::cout << "Frame " << Frame << "\n";
    pho::api::PhoXiTimeout timeout;
    pcl::PointCloud<pcl::PointXYZ>::Ptr PCLCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPCLCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ> MyPCLCloud;
    int count = 0;

    Frame->ConvertTo(MyPCLCloud);

    *PCLCloud = MyPCLCloud;

    for (size_t i = 0; i < MyPCLCloud.points.size(); ++i)
    {
        if(!(MyPCLCloud.points[i].z))
            count ++;
    }
    std::cout << "zero points: " << count << std::endl;
    std::cout << "Number of points in PCL Cloud : " << MyPCLCloud.points.size()<< std::endl;
    ///todo pclCloud transformation(); needed current robot joint states
    matrixTransformation(tempJointStates, PCLCloud, transformedPCLCloud); ///todo: function parameters => joint states[5], original pclCLoud, transformed pclCLoud.
    convertToOctomap(transformedPCLCloud);
//    convertToOctomap(PCLCloud);
}

void convertToOctomap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &PCLCloud){
    std::cout << "size: " << PCLCloud->points.size() << std::endl;
    for (size_t i = 0; i < PCLCloud->points.size(); ++i) {
        pcl::PointXYZ point = PCLCloud->points[i];
        tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }
    tree.updateInnerOccupancy();
}

void matrixTransformation(const double *joints,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceCloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &transformedCloud){
    Eigen::Matrix4f tempTransform[7];
    Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tempTransformMatrix = Eigen::Matrix4f::Identity();
    for(int idx=0; idx<6; idx++)
    {
        tempTransform[idx] = Eigen::Matrix4f::Identity();
    }
//    float theta = M_PI/2; // The angle of rotation in radians
    float theta = 0; // The angle of rotation in radians

    std::cout << joints[0] << ", " << joints[1] << ", " << joints[2] << ", " << joints[3] << ", " << joints[4] << ", " << joints[5] << std::endl;
    ///transformation matrix world to joint1 0.1525
    tempTransform[0] <<   std::cos(joints[0]), -std::sin(joints[0]),  0,      0,
                          std::sin(joints[0]),  std::cos(joints[0]),  0,      0,
                                               0,                       0,  1, 0.1525,
                                               0,                       0,  0,      1;

    ///transformation matrix joint1 to joint2 0.0345
    tempTransform[1] <<  std::cos(joints[1]),  0, std::sin(joints[1]),      0,
                                              0,  1,                      0, 0.0345,
                        -std::sin(joints[1]),  0, std::cos(joints[1]),      0,
                                              0,  0,                      0,      1;


    ///transformation matrix joint2 to joint3 0.62
    tempTransform[2] <<  std::cos(joints[2]),  0, std::sin(joints[2]),    0,
                                              0,  1,                      0,    0,
                        -std::sin(joints[2]),  0, std::cos(joints[2]), 0.62,
                                              0,  0,                      0,     1;

    ///transformation matrix joint3 to joint4 0.559
    tempTransform[3] <<   std::cos(joints[3]), -std::sin(joints[3]),  0,     0,
                          std::sin(joints[3]),  std::cos(joints[3]),  0,     0,
                                               0,                       0,  1, 0.559,
                                               0,                       0,  0,      1;

    ///transformation matrix joint4 to joint5 0
    tempTransform[4] <<  std::cos(joints[4]),  0, std::sin(joints[4]), 0,
                                              0,  1,                      0, 0,
                        -std::sin(joints[4]),  0, std::cos(joints[4]), 0,
                                              0,  0,                      0, 1;

    ///transformation matrix joint5 to joint6 0.121
    tempTransform[5] <<   std::cos(joints[5]), -std::sin(joints[5]),  0,     0,
                          std::sin(joints[5]),  std::cos(joints[5]),  0,     0,
                                               0,                       0,  1, 0.121,
                                               0,                       0,  0,     1;

    ///transformation matrix joint6 to camera
    tempTransform[6] <<   1, 0,  0,     0,
                          0, 0, -1, -0.14,
                          0, 1,  0,  0.08,
                          0, 0,  0,     1;

    transformMatrix = tempTransform[0]*
                      tempTransform[1]*
                      tempTransform[2]*
                      tempTransform[3]*
                      tempTransform[4]*
                      tempTransform[5]*
                      tempTransform[6];

//    transformMatrix = tempTransform[5]*
//                      tempTransform[4]*
//                      tempTransform[3]*
//                      tempTransform[2]*
//                      tempTransform[1]*
//                      tempTransform[0];
//    transformMatrix = tempTransform[1] ;

//    tempTransformMatrix << 1, 0, 0, 0,
//                           0, 0, 1, 0,
//                           0, -1, 0, 0,
//                           0, 0, 0, 1;

    std::cout << transformMatrix << std::endl;

    pcl::transformPointCloud (*sourceCloud, *transformedCloud, transformMatrix);
//    pcl::transformPointCloud (*sourceCloud, *transformedCloud, tempTransformMatrix);
}

void startSoftwareTriggerExample(pho::api::PPhoXi &PhoXiDevice) {
    if (PhoXiDevice->isAcquiring()) {
        // Stop acquisition to change trigger mode
        PhoXiDevice->StopAcquisition();
    }

    PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
    std::cout << "Software trigger mode was set" << std::endl;
    PhoXiDevice->ClearBuffer();
    PhoXiDevice->StartAcquisition();
    if (!PhoXiDevice->isAcquiring()) {
        std::cout << "Your device could not start acquisition!" << std::endl;
        return;
    }

    std::cout << "Triggering the frame" << std::endl;

    int tempNum=0;
    bool isRunning = true;
    while(isRunning)
    {
        std::cout << "Enter input: " ;
        std::cin >> tempNum;
        switch(tempNum)
        {
        case 1: ///front
        {
            int FrameID = PhoXiDevice->TriggerFrame();
            if (FrameID < 0)
            {
                // If negative number is returned trigger was unsuccessful
                std::cout << "Trigger was unsuccessful! code=" << FrameID << std::endl;
            }
            else
            {
                std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
            }
            std::cout << "Waiting for frame " << std::endl;
            pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
            tempJointStates[0] = -27.89 * DEGREE2RAD;
            tempJointStates[1] =   6.44 * DEGREE2RAD;
            tempJointStates[2] = 108.26 * DEGREE2RAD;
            tempJointStates[3] = -25.8  * DEGREE2RAD;
            tempJointStates[4] =  54.91 * DEGREE2RAD;
            tempJointStates[5] = 164.33 * DEGREE2RAD;

            if (Frame)
            {
                convertToPCL(Frame);
            }
            else
            {
                std::cout << "Failed to retrieve the frame!" << std::endl;
            }
            break;
        }
        case 2: ///left
        {
            int FrameID = PhoXiDevice->TriggerFrame();
            if (FrameID < 0)
            {
                // If negative number is returned trigger was unsuccessful
                std::cout << "Trigger was unsuccessful! code=" << FrameID << std::endl;
            }
            else
            {
                std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
            }
            std::cout << "Waiting for frame " << std::endl;
            pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
            tempJointStates[0] =  14.94 * DEGREE2RAD;
            tempJointStates[1] =  60.92 * DEGREE2RAD;
            tempJointStates[2] =  20.20 * DEGREE2RAD;
            tempJointStates[3] =  -3.37 * DEGREE2RAD;
            tempJointStates[4] =  78.91 * DEGREE2RAD;
            tempJointStates[5] = 108.98 * DEGREE2RAD;

            if (Frame)
            {
                convertToPCL(Frame);
            }
            else
            {
                std::cout << "Failed to retrieve the frame!" << std::endl;
            }
            break;
        }
        case 3: ///back
        {
            int FrameID = PhoXiDevice->TriggerFrame();
            if (FrameID < 0)
            {
                // If negative number is returned trigger was unsuccessful
                std::cout << "Trigger was unsuccessful! code=" << FrameID << std::endl;
            }
            else
            {
                std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
            }
            std::cout << "Waiting for frame " << std::endl;
            pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
            tempJointStates[0] = 58.16 * DEGREE2RAD;
            tempJointStates[1] = 64.02 * DEGREE2RAD;
            tempJointStates[2] = 14.44 * DEGREE2RAD;
            tempJointStates[3] = 12.04 * DEGREE2RAD;
            tempJointStates[4] = 80.84 * DEGREE2RAD;
            tempJointStates[5] = 60.52 * DEGREE2RAD;

            if (Frame)
            {
                convertToPCL(Frame);
            }
            else
            {
                std::cout << "Failed to retrieve the frame!" << std::endl;
            }
            break;
        }
        case 4:
        {
            int FrameID = PhoXiDevice->TriggerFrame();
            if (FrameID < 0)
            {
                // If negative number is returned trigger was unsuccessful
                std::cout << "Trigger was unsuccessful! code=" << FrameID << std::endl;
            }
            else
            {
                std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
            }
            std::cout << "Waiting for frame " << std::endl;
            pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
            tempJointStates[0] = 60.0 * DEGREE2RAD;
            tempJointStates[1] = 60.0 * DEGREE2RAD;
            tempJointStates[2] = 60.0 * DEGREE2RAD;
            tempJointStates[3] = 60.0 * DEGREE2RAD;
            tempJointStates[4] = 60.0 * DEGREE2RAD;
            tempJointStates[5] = 60.0 * DEGREE2RAD;
            if (Frame)
            {
                convertToPCL(Frame);
            }
            else
            {
                std::cout << "Failed to retrieve the frame!" << std::endl;
            }
            break;
        }

        case 5:
        {
            int FrameID = PhoXiDevice->TriggerFrame();
            if (FrameID < 0)
            {
                // If negative number is returned trigger was unsuccessful
                std::cout << "Trigger was unsuccessful! code=" << FrameID << std::endl;
            }
            else
            {
                std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
            }
            std::cout << "Waiting for frame " << std::endl;
            pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
            tempJointStates[0] = 0;
            tempJointStates[1] = 0;
            tempJointStates[2] = 0;
            tempJointStates[3] = 0;
            tempJointStates[4] = 0;
            tempJointStates[5] = 0;

            if (Frame)
            {
                convertToPCL(Frame);
            }
            else
            {
                std::cout << "Failed to retrieve the frame!" << std::endl;
            }
            break;
        }
        case 99:
        {
            tree.writeBinary("3Dshoes_debug4.bt"); // OctoMap 파일 저장
            std::cout << "end" << std::endl;
            isRunning = false;
            break;
        }
        default:
        {
            std::cout << "Invalid input. If you want to terminate enter '2'. " << std::endl;
            break;
        }
        }
    }
    PhoXiDevice->StopAcquisition();
}