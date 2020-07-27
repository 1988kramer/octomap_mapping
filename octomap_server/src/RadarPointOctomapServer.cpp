#include <octomap_server/RadarPointOctomapServer.h>

RadarPointOctomapServer::RadarPointOctomapServer(ros::NodeHandle private_nh_)
: OctomapServer<pcl::PointXYZI, octomap::OcTree>(private_nh_),
  m_useLocalMapping(false),
  m_azimuthFov(M_PI/4.0),
  m_elevationFov(M_PI/8.0),
  m_numScansInWindow(10),
  m_binWidth(0.070)
{
  private_nh.param("local_mapping", m_useLocalMapping, m_useLocalMapping);
  private_nh.param("num_scans_in_window", m_numScansInWindow, m_numScansInWindow);
  private_nh.param("bin_width", m_binWidth, m_binWidth);
  private_nh.param("sensor_model/azimuth_fov", m_azimuthFov, m_azimuthFov);
  private_nh.param("sensor_model/elevation_fov", m_elevationFov, m_elevationFov);

  double rayAngle = 2.0 * atan((0.5 * m_res) / m_maxRange);// angle between two adjacent rays
  size_t num_azimuth_bins = 2.0 * m_azimuthFov / rayAngle;
  size_t num_elevation_bins = 2.0 * m_elevationFov / rayAngle;

  m_radarRays.resize(num_azimuth_bins);
  for (int i = 0; i < num_azimuth_bins; i++)
    m_radarRays[i].resize(num_elevation_bins);

  for (int i = 0; i < num_azimuth_bins; i++)
  {
    for (int j = 0; j < num_elevation_bins; j++)
    {
      double el_angle = double(j) * rayAngle - m_elevationFov;
      double az_angle = double(i) * rayAngle - m_azimuthFov;
      Eigen::Vector3d ray(cos(az_angle) * cos(el_angle),
                          sin(az_angle) * cos(el_angle),
                          sin(el_angle));
      m_radarRays[i][j] = ray;
    }
  }
}


void RadarPointOctomapServer::filterReflections(const PCLPointCloud& cloud,
                                                PCLPointCloud& out_cloud)
{
  // detect point clusters
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloudPtr(new const pcl::PointCloud<point_t>(cloud));
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<point_t>());
  tree->setInputCloud(cloudPtr);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.2);
  if (m_useLocalMapping)
    ec.setMinClusterSize(3);
  else
    ec.setMinClusterSize(2);
  ec.setMaxClusterSize(20);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloudPtr);
  ec.extract(cluster_indices);

  out_cloud.clear();

  // determine if clusters are roughly on same ray from sensor
  // if so cluster is likely reflections
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
    it != cluster_indices.end(); it++)
  {
    std::vector<int> indices_copy = it->indices;
    bool replace_with_mean = false;
    if (indices_copy.size() >= 3)
    {
      // get unit ray pointing toward each point in the cluster
      std::vector<Eigen::Vector3d> rays;
      for (std::vector<int>::const_iterator pit = it->indices.begin();
        pit != it->indices.end(); pit++)
      {
        Eigen::Vector3d unit_ray(cloud.points[*pit].x,
         cloud.points[*pit].y,
         cloud.points[*pit].z);
        unit_ray.normalize();
        rays.push_back(unit_ray);
      }
      // determine the angle between each pair of points
      Eigen::MatrixXd angles(rays.size(), rays.size());
      angles.setZero();

      for (int i = 0; i < rays.size(); i++)
      {
        for (int j = i+1; j < rays.size(); j++)
        {
          angles(i,j) = std::fabs(acos(rays[i].dot(rays[j])));
          angles(j,i) = angles(i,j);
        }
      }
      // remove outliers
      int cluster_idx = 0;
      for (int i = 0; i < angles.cols(); i++)
      {
        double min_angle = 10.0;
        for (int j = 0; j < angles.rows(); j++)
        {
          if (i != j && angles(i,j) < min_angle) 
            min_angle = angles(i,j);
        }
        if (min_angle > 2.0 * m_binWidth)
        {
          indices_copy.erase(indices_copy.begin() + cluster_idx);
        }
        else
        {
          cluster_idx++;
        }
      }
      replace_with_mean = indices_copy.size() > 3;
    }

    // if max angle is less than bin width, add the 
    // mean of the points to the new cloud
    if (replace_with_mean)
    {
      pcl::PointXYZI mean_point;
      mean_point.x = 0.0;
      mean_point.y = 0.0;
      mean_point.z = 0.0;
      mean_point.intensity = 0.0;
      for (std::vector<int>::const_iterator pit = indices_copy.begin();
        pit != indices_copy.end(); pit++)
      {
        mean_point.x += cloud.points[*pit].x;
        mean_point.y += cloud.points[*pit].y;
        mean_point.z += cloud.points[*pit].z;
        mean_point.intensity += cloud.points[*pit].intensity;
      }
      mean_point.x /= indices_copy.size();
      mean_point.y /= indices_copy.size();
      mean_point.z /= indices_copy.size();
      mean_point.intensity /= indices_copy.size();

      out_cloud.points.push_back(mean_point);
    }
    else // add the individual points to the cloud
    {
      for (std::vector<int>::const_iterator pit = it->indices.begin();
        pit != it->indices.end(); pit++)
      {
        out_cloud.points.push_back(cloud.points[*pit]);
      }
    }
  }
}

void RadarPointOctomapServer::insertRadarScanToDeque(const tf::StampedTransform& sensorPoseTf,
                                     const PCLPoint& pointCloud)
{
  std_srvs::Empty::Request req; 
  std_srvs::Empty::Response resp;
  resetSrv(req, resp);
  Eigen::Matrix4f sensorPose;
  pcl_ros::transformAsMatrix(sensorPoseTf, sensorPose);
  m_pointClouds.push_front(std::make_pair(pointCloud, sensorPose));
  if (m_pointClouds.size() > m_numScansInWindow)
  {
    m_pointClouds.pop_back();
  }
  Eigen::Matrix4f initial_pose = m_pointClouds.front().second;
  pcl::PointCloud<pcl::PointXYZI> composed_cloud;
  for (int i = 0; i < m_pointClouds.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI> tfCloud;
    pcl::transformPointCloud(m_pointClouds[i].first, tfCloud, initial_pose.inverse() * m_pointClouds[i].second);
    composed_cloud += tfCloud;
  }
  insertRadarScanToMap(composed_cloud, Eigen::Matrix4f::Identity());
}


void RadarPointOctomapServer::getCellsInFov(const Eigen::Matrix4f& sensorPose,
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
        cells.insert(m_keyRay.begin(), m_keyRay.end());

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

void RadarPointOctomapServer::insertRadarScanToMap(const PCLPointCloud& pointCloud,
                                        const Eigen::Matrix4f& sensorPose)
{

  KeySet free_cells;
  KeySet occupied_cells;

  getCellsInFov(sensorPose, free_cells);
  
  // remove cells that contain targets from the free cell set 
  // and add them to the occupied cell set
  for (PCLPointCloud::const_iterator it = pointCloud.begin(); 
       it != pointCloud.end(); it++)
  {
    point3d target(it->x, it->y, it->z);
    octomap::OcTreeKey targetKey;
    if(m_octree->coordToKeyChecked(target, targetKey))
    {
      KeySet::iterator key_it = free_cells.find(targetKey);
      if (key_it != free_cells.end())
      {
        free_cells.erase(key_it);
        occupied_cells.insert(targetKey);
      }
    }
    else
    {
      ROS_ERROR_STREAM("could not create Key for radar target at " << target);
    }
  }

  // update occupancy probability for free cells
  for (KeySet::iterator it = free_cells.begin(); 
    it != free_cells.end(); it++)
  {
    m_octree->updateNode(*it, false);
  }

  // update occupancy probability for occupied cells
  for (KeySet::iterator it = occupied_cells.begin(); 
    it != occupied_cells.end(); it++)
  {
    m_octree->updateNode(*it, true);
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

  #ifdef COLOR_OCTOMAP_SERVER
  if (colors)
  {
    delete[] colors;
    colors = NULL;
  }
  #endif
}


void RadarPointOctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
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


  PCLPointCloud pc_filtered;
  filterReflections(pc, pc_filtered);

  if (m_useLocalMapping)
  {
    insertRadarScanToDeque(sensorToWorldTf, pc_filtered);
  }
  else
  {
    pcl::transformPointCloud(pc_filtered, pc_filtered, sensorToWorld);
    insertRadarScanToMap(pc_filtered, sensorToWorld);
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu pts, %f sec)", pc.size(), total_elapsed);
  
  publishAll(cloud->header.stamp);
}


void RadarPointOctomapServer::publishAll(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
  bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
  bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
  bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
  bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
  m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

  // init markers for free space:
  visualization_msgs:publishA:MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_treeDepth+1);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  // init pointcloud:
  PCLPointCloud pclCloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
      end = m_octree->end(); it != end; ++it)
  {
    bool inUpdateBBX = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (inUpdateBBX)
      handleNodeInBBX(it);

    if (m_octree->isNodeOccupied(*it)){
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();

        // Ignore speckles in the map:
        if (m_filterSpeckles && (it.getDepth() == m_treeDepth) && isSpeckleNode(it.getKey())){
          ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        } // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);
        if (inUpdateBBX)
          handleOccupiedNodeInBBX(it);


        //create marker:
        if (publishMarkerArray){
          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
          if (m_useHeightMap){
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }
        }

        // insert into pointcloud:
        if (publishPointCloud) {
          pclCloud.push_back(PCLPoint(x, y, z));
        }

      }
    } else { // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ)
      {
        handleFreeNode(it);
        if (inUpdateBBX)
          handleFreeNodeInBBX(it);

        if (m_publishFreeSpace){
          double x = it.getX();
          double y = it.getY();

          //create marker for free space:
          if (publishFreeMarkerArray){
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }

      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);

  // finish MarkerArray:
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      if (m_sensorModel.compare("radar_point") == 0 && m_useLocalMapping)
        occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      else
        occupiedNodesVis.markers[i].header.frame_id = m_baseFrameId;
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      if (!m_useColoredMap)
        occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_markerPub.publish(occupiedNodesVis);
  }


  // finish FreeMarkerArray:
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "map";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;
      freeNodesVis.markers[i].color = m_colorFree;


      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_fmarkerPub.publish(freeNodesVis);
  }


  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_pointCloudPub.publish(cloud);
  }

  if (publishBinaryMap)
    publishBinaryOctoMap(rostime);

  if (publishFullMap)
    publishFullOctoMap(rostime);


  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);

}