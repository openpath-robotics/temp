//
// Created by jh on 24. 11. 25.
//
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#define DEGREE2RAD 3.141592/180.0
// This function displays the help
void
showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

// This is the main function
int
main (int argc, char** argv)
{

    // Show help
    if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
        showHelp (argv[0]);
        return 0;
    }

    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

    if (filenames.size () != 1)  {
        filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

        if (filenames.size () != 1) {
            showHelp (argv[0]);
            return -1;
        } else {
            file_is_pcd = true;
        }
    }

    // Load file | Works with PCD and PLY files
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    if (file_is_pcd) {
        if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
            showHelp (argv[0]);
            return -1;
        }
    } else {
        if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
            showHelp (argv[0]);
            return -1;
        }
    }

    /* Reminder: how transformation matrices work :

             |-------> This column is the translation
      | 1 0 0 x |  \
      | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
      | 0 0 1 z |  /
      | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

      METHOD #1: Using a Matrix4f
      This is the "manual" method, perfect to understand but error prone !
    */
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = M_PI/2; // The angle of rotation in radians
    transform_1 (0,0) = std::cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = std::cos (theta);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1 (0,3) = 0.0;

    // Print the transformation
    printf ("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    /*  METHOD #2: Using a Affine3f
      This method is easier and less error prone
    */
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 0.0, 0.0, 0.0;

    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    ///
    Eigen::Matrix4f tempTransform[7];
    Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tempTransformMatrix = Eigen::Matrix4f::Identity();
    double joints[6];
//    joints[0] = -27.89 * DEGREE2RAD;
//    joints[1] =   6.44 * DEGREE2RAD;
//    joints[2] = 108.26 * DEGREE2RAD;
//    joints[3] = -25.8  * DEGREE2RAD;
//    joints[4] =  54.91 * DEGREE2RAD;
//    joints[5] = 164.33 * DEGREE2RAD;
////
//    joints[0] =  14.94 * DEGREE2RAD;
//    joints[1] =  60.92 * DEGREE2RAD;
//    joints[2] =  20.20 * DEGREE2RAD;
//    joints[3] =  -3.37 * DEGREE2RAD;
//    joints[4] =  78.91 * DEGREE2RAD;
//    joints[5] = 108.98 * DEGREE2RAD;
//
    joints[0] = 58.16 * DEGREE2RAD;
    joints[1] = 64.02 * DEGREE2RAD;
    joints[2] = 14.44 * DEGREE2RAD;
    joints[3] = 12.04 * DEGREE2RAD;
    joints[4] = 80.84 * DEGREE2RAD;
    joints[5] = 60.52 * DEGREE2RAD;
//
    for(int idx=0; idx<6; idx++)
    {
        tempTransform[idx] = Eigen::Matrix4f::Identity();
    }
//    float theta = M_PI/2; // The angle of rotation in radians

    std::cout << joints[0] << ", " << joints[1] << ", " << joints[2] << ", " << joints[3] << ", " << joints[4] << ", " << joints[5] << std::endl;
    ///transformation matrix world to joint1 0.1525
    tempTransform[0] <<   std::cos(joints[0]),  -std::sin(joints[0]),  0,      0,
        std::sin(joints[0]),  std::cos(joints[0]),  0,      0,
        0,                       0,  1, -0.1525,
        0,                       0,  0,      1;

    ///transformation matrix joint1 to joint2 0.0345
    tempTransform[1] <<  std::cos(joints[1]),  0, -std::sin(joints[1]),      0,
        0,  1,                      0, -0.0345,
        std::sin(joints[1]),  0, std::cos(joints[1]),      0,
        0,  0,                      0,      1;


    ///transformation matrix joint2 to joint3 0.62
    tempTransform[2] <<  std::cos(joints[2]),  0, -std::sin(joints[2]),    0,
        0,  1,                      0,    0,
        std::sin(joints[2]),  0, std::cos(joints[2]), -0.62,
        0,  0,                      0,     1;

    ///transformation matrix joint3 to joint4 0.559
    tempTransform[3] <<   std::cos(joints[3]),  -std::sin(joints[3]),  0,     0,
        std::sin(joints[3]),  std::cos(joints[3]),  0,     0,
        0,                       0,  1, -0.559,
        0,                       0,  0,      1;

    ///transformation matrix joint4 to joint5 0
    tempTransform[4] <<  std::cos(joints[4]),  0, -std::sin(joints[4]), 0,
        0,  1,                      0, 0,
        std::sin(joints[4]),  0, std::cos(joints[4]), 0,
        0,  0,                      0, 1;

    ///transformation matrix joint5 to joint6 0.121
    tempTransform[5] <<   std::cos(joints[5]),  -std::sin(joints[5]),  0,     0,
        std::sin(joints[5]),  std::cos(joints[5]),  0,     0,
        0,                       0,  1, -0.121,
        0,                       0,  0,     1;

    ///transformation matrix joint6 to camera
    tempTransform[6] <<   1, 0,  0,     0,
        0, 0, -1, 0.14,
        0, 1,  0,  -0.08,
        0, 0,  0,     1;

    transformMatrix = tempTransform[0]*
        tempTransform[1]*
        tempTransform[2]*
        tempTransform[3]*
        tempTransform[4]*
        tempTransform[5]*
        tempTransform[6];

    ///

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transformMatrix);

    // Visualization
    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
             "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }

    return 0;
}