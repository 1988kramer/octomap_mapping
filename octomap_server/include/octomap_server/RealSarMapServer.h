#ifndef OCTOMAP_SERVER_REALSARMAPSERVER_H_
#define OCTOMAP_SERVER_REALSARMAPSERVER_H_

#include "octomap_server/OctomapServer.h"

namespace octomap_server
{
  class RealSarMapServer: public OctomapServer<pcl::PointXYZI, octomap::MeanOcTree>
  {
  public:
    RealSarMapServer(ros::NodeHandle private_nh);

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

  protected:

    /**
      * @brief update opccupancy map with a dense radar image in polar coordinates
      * @param msg The dense pointcloud representing the radar image in the global frame
      * @param sensorPose The pose of the radar sensor in the global frame
      */
    void insertRadarImageToMap(const pcl::PointCloud<pcl::PointXYZI>& pointcloud, 
                               const Eigen::Matrix4f& sensorPose);

    /**
      * @brief uses barycentric interpolation to calculate the intensity of a query
      *        point from the intensities and inverse distances to a set of
      *        surrounding points
      * @param[in] intensities the intensities of the surrounding points
      * @param[in] inverse_dists the inverse distances to the surrounding points
      * @return the interpolated intensity of the query point
      */
    float barycentricInterpolate(const std::vector<float> &intensities,
                                 const std::vector<float> &inverse_dists);

    /**
      * @brief finds the set of cells currently in the sensor's field of view
      * @param[in] the sensor's current pose
      * @param[out] the set of cell keys within the sensor's field of view
      */
    void getCellsInFov(const Eigen::Matrix4f& sensorPose, octomap::KeySet& cells);


    double m_azimuthFov;
    double m_elevationFov;
    std::vector<std::vector<Eigen::Vector3d>> m_radarRays;

  }
}

#endif