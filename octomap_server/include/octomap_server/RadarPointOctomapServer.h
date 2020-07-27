#ifndef OCTOMAP_SERVER_RADARPOINTOCTOMOPSERVER_H_
#define OCTOMAP_SERVER_RADARPOINTOCTOMOPSERVER_H_

#include "octomap_server/OctomapServer.h"

namespace octomap_server
{
  class RadarPointOctomapServer: public OctomapServer<pcl::PointXYZI, octomap::OcTree>
  {
  public:
    RadarPointOctomapServer(ros::NodeHandle private_nh_);

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

  protected:
    void insertRadarScanToMap(const PCLPointCloud& pointCloud,
                              const Eigen::Matrix4f& sensorPose);

    /**
    * @brief finds the set of cells currently in the sensor's field of view
    * @param[in] the sensor's current pose
    * @param[out] the set of cell keys within the sensor's field of view
    */
    void getCellsInFov(const Eigen::Matrix4f& sensorPose, octomap::KeySet& cells);

    /**
    * @brief filters multipath reflections from input radar point cloud
    * @param[in] cloud The raw pointcloud
    * @param[out] out_cloud filtered point cloud
    */
    void filterReflections(const PCLPointCloud& cloud, 
                           PCLPointCloud& out_cloud);

    /**
    * @brief Adds the most recent scan to map window and removes the oldest if necessary
    * @param[in] sensorPoseTf The transform describing the sensor's estimated 
    *                          pose in the world frame
    * @param[in] pointCloud The point cloud in the sensor frame
    */
    void insertRadarScanToDeque(const tf::StampedTransform& sensorPoseTf,
                                const PCLPointCloud& pointCloud);

    void publishAll(const ros::Time& rostime = ros::Time::now());


    std::deque<std::pair<pcl::PointCloud<pcl::PointXYZI>,Eigen::Matrix4f> > m_pointClouds;
    std::vector<std::vector<Eigen::Vector3d>> m_radarRays;
    int m_numScansInWindow;
    bool m_useLocalMapping;
    double m_azimuthFov;
    double m_elevationFov;
    double m_binWidth;
  }
}

#endif