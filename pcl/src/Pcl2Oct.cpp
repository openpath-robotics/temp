//
// Created by jh on 24. 11. 27.
//
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"

int main(int argc, char** argv) {
    // PCL 포인트 클라우드 로드
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("input.ply", *cloud) == -1) {
        PCL_ERROR("Couldn't read the PLY file\n");
        return -1;
    }

    // OctoMap 생성
    double resolution = 0.1; // OctoMap의 해상도
    octomap::OcTree tree(resolution);

    // PCL 포인트 클라우드를 OctoMap으로 변환

    for (auto& point : cloud->points) {
        tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }

    // OctoMap 저장
    tree.updateInnerOccupancy();
    tree.writeBinary("test.bt"); // OctoMap 파일 저장

    return 0;
}