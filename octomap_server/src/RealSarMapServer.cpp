#include <octomap_server/RealSarMapServer.h>

using namespace octomap;

namespace octomap_server
{

RealSarMapServer::RealSarMapServer(ros::NodeHandle private_nh_)
: OctomapServer<pcl::PointXYZI, octomap::MeanOcTree>(private_nh_),
  m_azimuthFov(M_PI/4.0),
  m_elevationFov(M_PI/8.0),
  m_intensityThreshold(0.0)
{
  private_nh_.param("sensor_model/azimuth_fov", m_azimuthFov, m_azimuthFov);
  private_nh_.param("sensor_model/elevation_fov", m_elevationFov, m_elevationFov);
  private_nh_.param("intensity_threshold", m_intensityThreshold, m_intensityThreshold);
  m_octree->setIntensityThreshold(m_intensityThreshold);

  double rayAngle = 2.0 * atan((0.5 * m_res) / m_maxRange);// angle between two adjacent rays
  size_t num_azimuth_bins = 2.0 * m_azimuthFov / rayAngle;
  size_t num_elevation_bins = 2.0 * m_elevationFov / rayAngle;

  m_radarRays.resize(num_azimuth_bins);
  for (int i = 0; i < num_azimuth_bins; i++)
    m_radarRays[i].resize(num_elevation_bins);

  for (int i = 0; i < num_azimuth_bins; i++)
  {
    double az_angle = double(i) * rayAngle - m_azimuthFov;
    for (int j = 0; j < num_elevation_bins; j++)
    {
      double el_angle = double(j) * rayAngle - m_elevationFov;
      Eigen::Vector3d ray(cos(az_angle) * cos(el_angle),
                          sin(az_angle) * cos(el_angle),
                          sin(el_angle));
      m_radarRays[i][j] = ray;
    }
  }
}


void RealSarMapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  ros::WallTime startTime = ros::WallTime::now();

  tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  PCLPointCloud pc;
  pcl::fromROSMsg(*cloud, pc);

  pcl::transformPointCloud(pc, pc, sensorToWorld);
  insertRadarImageToMap(pc, sensorToWorld);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu pts, %f sec)", pc.size(), total_elapsed);
  
  publishAll(cloud->header.stamp);
}


float RealSarMapServer::barycentricInterpolate(const std::vector<float> &intensities,
                                            const std::vector<float> &inverse_dists)
{
  float sum_inverse_dists = 0.0;
  for (int i = 0; i < inverse_dists.size(); i++)
    sum_inverse_dists += inverse_dists[i];

  float out_intensity = 0.0;
  for (int i = 0; i < intensities.size(); i++)
    out_intensity += intensities[i] * inverse_dists[i] / sum_inverse_dists;

  return out_intensity;
}


void RealSarMapServer::insertRadarImageToMap(const PCLPointCloud& pointcloud,
                                          const Eigen::Matrix4f& sensorPose)
{
  // Create KD tree from the radar pointcloud
  //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(pointcloud);
  pcl::KdTreeFLANN<PCLPoint> kdtree;
  pcl::PointCloud<PCLPoint>::ConstPtr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(pointcloud));
  //ROS_ERROR_STREAM("setting input cloud");
  kdtree.setInputCloud(cloud_ptr, NULL);
  //ROS_ERROR_STREAM("getting cells in fov");
  KeySet cell_keys;
  getCellsInFov(sensorPose, cell_keys);
  //ROS_ERROR_STREAM("updating cell values");
  for (KeySet::iterator it = cell_keys.begin();
       it != cell_keys.end(); it++)
  {
    //ROS_ERROR_STREAM("getting query point");
    point3d cell_coord = m_octree->keyToCoord(*it);
    pcl::PointXYZI query_point;
    query_point.x = cell_coord.x();
    query_point.y = cell_coord.y();
    query_point.z = cell_coord.z();
    query_point.intensity = 0.0;
    //ROS_ERROR_STREAM("finding surrounding radar image voxels");
    // Get eight surrounding points in radar image
    int K=3;
    std::vector<int> indices(K);
    std::vector<float> sqr_distances(K);
    kdtree.nearestKSearch(query_point, K, indices, sqr_distances);


    // determine if resultant points form box around given map cell
    // center the 8 points and check the positivity/negativity of each coordinate
    //ROS_ERROR_STREAM("checking if voxels surround query point");
    std::vector<bool> corners(K,false);
    std::vector<float> inverse_dists(K);
    std::vector<float> intensities(K);
    for (int i = 0; i < indices.size(); i++)
    {
      /*
      //ROS_ERROR_STREAM("centering local point");
      pcl::PointXYZI local_point;
      local_point.x = pointcloud[indices[i]].x - query_point.x;
      local_point.y = pointcloud[indices[i]].y - query_point.y;
      local_point.z = pointcloud[indices[i]].z - query_point.z;
      local_point.intensity = pointcloud[indices[i]].intensity;

      //ROS_ERROR_STREAM("finding local point location");
      unsigned char c = 0;
      if (local_point.x > 0.0)
        c |= 1 << 0;
      if (local_point.y > 0.0)
        c |= 1 << 1;
      if (local_point.z > 0.0)
        c |= 1 << 2;
      //ROS_ERROR_STREAM("updating corners for point " << i << " at index " << int(c));
      corners[int(c)] = true;
      */
      intensities[i] = pointcloud[indices[i]].intensity;
      inverse_dists[i] = 1.0 / sqrt(sqr_distances[i]);
    }
    if (true) //std::find(corners.begin(), corners.end(), false) == corners.end())
    {      
      float intensity = barycentricInterpolate(intensities, inverse_dists);
      //ROS_ERROR_STREAM("updating map voxel " << cell_coord << " with intensity " << intensity);
      // convert interpolated intensity to an occupancy probability update
      m_octree->addObservation(*it,intensity);
    }
    
  }
  octomap::point3d minPt, maxPt;
  ROS_DEBUG_STREAM("Bounding box keys (before): "
    << m_updateBBXMin[0] << " " << m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / "
    << m_updateBBXMax[0] << " " << m_updateBBXMax[1] << " " << m_updateBBXMax[2]);
  minPt = m_octree->keyToCoord(m_updateBBXMin);
  maxPt = m_octree->keyToCoord(m_updateBBXMax);
  ROS_DEBUG_STREAM("Bounding box keys (after): "
    << m_updateBBXMin[0] << " " << m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / "
    << m_updateBBXMax[0] << " " << m_updateBBXMax[1] << " " << m_updateBBXMax[2]);

  if (m_compressMap)
    m_octree->prune();
}

void RealSarMapServer::getCellsInFov(const Eigen::Matrix4f& sensorPose,
                                  KeySet& cells)
{
  for (int i = 0; i < m_radarRays.size(); i++)
  {
    for (int j = 0; j < m_radarRays[i].size(); j++)
    {
      Eigen::Vector4f sensorFrameRayStart(m_radarRays[i][j].x() * m_minRange,
                                          m_radarRays[i][j].y() * m_minRange,
                                          m_radarRays[i][j].z() * m_minRange,
                                          1.0);
      Eigen::Vector4f sensorFrameRayEnd(m_radarRays[i][j].x() * m_maxRange,
                                        m_radarRays[i][j].y() * m_maxRange,
                                        m_radarRays[i][j].z() * m_maxRange,
                                        1.0);
      Eigen::Vector4f globalFrameRayStart = sensorPose * sensorFrameRayStart;
      Eigen::Vector4f globalFrameRayEnd = sensorPose * sensorFrameRayEnd;
      point3d startPoint(globalFrameRayStart[0],
                         globalFrameRayStart[1],
                         globalFrameRayStart[2]);
      point3d endPoint(globalFrameRayEnd[0],
                       globalFrameRayEnd[1],
                       globalFrameRayEnd[2]);

      if (m_octree->computeRayKeys(startPoint, endPoint, m_keyRay))
      {
        cells.insert(m_keyRay.begin(), m_keyRay.end());
      }

      octomap::OcTreeKey endKey;
      if (m_octree->coordToKeyChecked(endPoint, endKey))
      {
        updateMinKey(endKey, m_updateBBXMin);
        updateMaxKey(endKey, m_updateBBXMax);
      }
      else
      {
        ROS_ERROR_STREAM("Could not generate key for radar fov point " << endPoint);
      }
    }
  }
}

} // end namespace octomap_server
