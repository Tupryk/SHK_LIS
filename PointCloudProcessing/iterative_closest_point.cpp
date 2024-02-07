#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>


int main()
{
    // Load point cloud data from files
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

    // Load point cloud data from files
    for (int i = 0; i < 8; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::string filename = "../data/point_cloud_" + std::to_string(i) + ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", filename.c_str());
            return (-1);
        }
        std::cout << "Loaded " << cloud->size() << " data points from " << filename << std::endl;
        clouds.push_back(cloud);
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>); // Initialize result cloud with the first input cloud
    *result_cloud = *clouds[0];

    // Perform ICP registration for each pair of consecutive point clouds
    for (size_t i = 1; i < clouds.size(); ++i)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(clouds[i]);
        icp.setInputTarget(result_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*aligned_cloud);

        std::cout << "Pair " << i << " registration has converged: " << icp.hasConverged()
                  << " score: " << icp.getFitnessScore() << std::endl;

        *result_cloud += *aligned_cloud; // Merge aligned_cloud with result_cloud
    }

    pcl::io::savePCDFileASCII("../data/result_cloud.pcd", *result_cloud);

    return (0);
}
