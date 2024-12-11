#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <iostream>
#include <sstream>
#include <fstream>

int main() {
    // Load the PLY file into a PointXYZRGB point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("shoes.ply", *cloud) == -1) {
        PCL_ERROR("Couldn't read the PLY file\n");
        return -1;
    }
    std::cout << "Loaded point cloud with " << cloud->size() << " points." << std::endl;

    // Compression settings
    double pointResolution = 0.01;    // Smallest distance between points
    double octreeResolution = 2.0;   // Voxel size
    bool doColorEncoding = true;     // Enable color compression
    bool showStatistics = true;      // Show compression details
    unsigned char iFrameRate = 30;   // Intra frame rate for streaming (optional)

    // Create a compressor
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> compressor(
        pcl::io::MANUAL_CONFIGURATION,
        showStatistics,
        pointResolution,
        octreeResolution,
        doColorEncoding,
        iFrameRate
    );

    // Compress the point cloud
    std::stringstream compressedData;
    compressor.encodePointCloud(cloud, compressedData);

    // Decompress the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr decompressedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    compressor.decodePointCloud(compressedData, decompressedCloud);

    // Save the decompressed point cloud to a PLY file
    pcl::io::savePLYFileBinary("output.ply", *decompressedCloud); // Save in binary PLY format
    std::cout << "Decompressed point cloud saved to 'output.ply' with " << decompressedCloud->size() << " points." << std::endl;

    return 0;
}