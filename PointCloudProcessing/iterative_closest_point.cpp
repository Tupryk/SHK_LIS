// #define VISUALIZATION_ENABLED

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Conditional compilation for visualization
#ifdef VISUALIZATION_ENABLED
#include <pcl/visualization/cloud_viewer.h>
#endif

// Constants for statistical outlier removal
const int mean_k = 60;  // Number of nearest neighbors to compute mean distance
const double std_dev = 1.5;  // Standard deviation multiplier

// Function to merge two point clouds using ICP with outlier removal
pcl::PointCloud<pcl::PointXYZ>::Ptr mergePointClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
                                                     bool filterOutliers = false)
{
    // Initialize the ICP object
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    // Set the input source and target clouds for ICP
    icp.setInputSource(cloud2);
    icp.setInputTarget(cloud1);

    // Align the source cloud to the target cloud (cloud1)
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*aligned_cloud);

    // Check if the ICP alignment has converged
    if (icp.hasConverged())
    {
        std::cout << "ICP converged with fitness score: " << icp.getFitnessScore() << std::endl;


        if(filterOutliers) {
            // Apply outlier removal to the aligned cloud
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(aligned_cloud);
            sor.setMeanK(mean_k);
            sor.setStddevMulThresh(std_dev);
            sor.filter(*aligned_cloud);
        }

        // Merge aligned_cloud with cloud1
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *merged_cloud = *cloud1 + *aligned_cloud;

        return merged_cloud;
    }
    else
    {
        std::cout << "ICP did not converge." << std::endl;
        return cloud1; // Return cloud1 unchanged
    }
}



// Function to merge point clouds using divide and conquer approach
pcl::PointCloud<pcl::PointXYZ>::Ptr mergePointCloudsDivideConquer(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds,
                                                                   int start, int end, bool filterOutliers = false)
{
    if (start == end)
        return clouds[start];
    
    else if (end - start == 1)
        return mergePointClouds(clouds[start], clouds[end], filterOutliers);
    
    else {
        int mid = (start + end) / 2;
        pcl::PointCloud<pcl::PointXYZ>::Ptr left_merged = mergePointCloudsDivideConquer(clouds, start, mid, filterOutliers);
        pcl::PointCloud<pcl::PointXYZ>::Ptr right_merged = mergePointCloudsDivideConquer(clouds, mid + 1, end, filterOutliers);
        return mergePointClouds(left_merged, right_merged, filterOutliers);
    }
}


int main() {
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

        std::cout << "Final transformation matrix:\n"
                  << icp.getFinalTransformation() << std::endl;

        std::cout << "Pair " << i << " registration has converged: " << icp.hasConverged()
                  << " score: " << icp.getFitnessScore() << std::endl;

        *result_cloud += *aligned_cloud; // Merge aligned_cloud with result_cloud
    }

    // Create a statistical outlier removal filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(result_cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_dev);

    // Filter outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered_cloud);

    pcl::io::savePCDFileASCII("../data/result_cloud.pcd", *result_cloud);
    pcl::io::savePCDFileASCII("../data/filtered_result_cloud.pcd", *filtered_cloud);
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_merged_cloud = mergePointCloudsDivideConquer(clouds, 0, clouds.size() - 1, false);
    pcl::io::savePCDFileASCII("../data/final_aligned_cloud.pcd", *final_merged_cloud);

    
    sor.setInputCloud(final_merged_cloud);
    // Filter outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_merged_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*final_merged_cloud_filtered);

    pcl::io::savePCDFileASCII("../data/final_aligned_cloud_filtered.pcd", *final_merged_cloud_filtered);

    // Visualization
    #ifdef VISUALIZATION_ENABLED
    pcl::visualization::CloudViewer viewer("Result");
    viewer.showCloud(filtered_cloud);
    while (!viewer.wasStopped()) {}
    #endif

    return 0;
}
