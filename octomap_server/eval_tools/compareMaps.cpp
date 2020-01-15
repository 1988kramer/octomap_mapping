#include <glog/logging.h>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#define foreach BOOST_FOREACH

typedef std::pair<double,pcl::KdTreeFLANN<pcl::PointXYZ>> stampedMap;
typedef std::pair<double,std::vector<double>> pointDistances;
typedef std::vector<pointDistances> distanceList;

// pulls measurements from a rosbag for every topic listed in topics vector
void getMeasurements(std::vector<std::vector<stampedMap>> &measurements, 
                     std::vector<std::string> topics,
                     std::string bagfile_name)
{
  // open rosbag and find requested topics
  rosbag::Bag bag;
  bag.open(bagfile_name, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  size_t num_topics = topics.size();


  // iterate through rosbag and transfer messages to their proper containers
  foreach(rosbag::MessageInstance const m, view)
  {
    std::string topic = m.getTopic();

    size_t topic_index = std::distance(topics.begin(), 
                                         find(topics.begin(), 
                                              topics.end(), 
                                              topic));

    sensor_msgs::PointCloud2::ConstPtr pcl2_msg = 
      m.instantiate<sensor_msgs::PointCloud2>();

    if (pcl2_msg != NULL)
    {
      // get timestamp
      double timestamp = pcl2_msg->header.stamp.toSec();

      // convert pointcloud2 message to pcl struct
      pcl::PCLPointCloud2 pcl2_cloud;
      pcl_conversions::toPCL(*pcl2_msg, pcl2_cloud);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(pcl2_cloud, *pcl_cloud);

      // if pointcloud is empty, skip to next
      if (pcl_cloud->size() == 0)
        continue;

      // create kd tree
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      kdtree.setInputCloud(pcl_cloud);

      // add to measurements vector
      measurements[topic_index].push_back(std::make_pair(timestamp,kdtree));
    }   
  }
  bag.close();  
}

std::string replaceSlashes(std::string &input)
{
  std::string result = input;
  for (int i = 0; i < input.size(); i++)
  {
    if (result[i] == '/')
      result[i] = '_';
  }
  return result;
}

void writeToCsv(std::vector<std::vector<stampedMap>> &meas,
                std::string &filename_prefix,
                std::vector<std::string> &topics)
{
  for (size_t i = 0; i < topics.size(); i++)
  {
    std::string filename = filename_prefix + replaceSlashes(topics[i]) + ".csv";
    std::ofstream out_file(filename, std::ios_base::trunc);

    
    out_file.close();
  }
}

// for every point in each pointcloud from each topic, get the distance to the
// nearest neighbor in every other topic's pointcloud at roughly the same time
void getDistanceLists(std::vector<std::vector<stampedMap>> &measurements,
                      std::vector<std::vector<distanceList>> &cloud_distances,
                      std::vector<std::string> &topics)
{
  size_t num_topics = measurements.size();
  size_t num_combinations = num_topics * (num_topics - 1);

  cloud_distances.clear();
  cloud_distances.resize(num_topics);
  for (size_t i = 0; i < num_topics; i++)
    cloud_distances[i].resize(num_topics);

  for (size_t i = 0; i < num_topics; i++)
  {
    for (size_t j = 0; j < num_topics; j++)
    {
      if (i != j)
      {
        LOG(ERROR) << "comparing map from topic " << topics[i] 
                   << " to map from topic " << topics[j];

        std::vector<stampedMap>::const_iterator ref_it = measurements[i].begin();
        std::vector<stampedMap>::const_iterator query_it = measurements[j].begin();

        // ensure reference pointclouds exist before the first query pointcloud
        while (query_it != measurements[j].end() 
          && ref_it->first > query_it->first) query_it++;

        // iterate over query point clouds and calculate score for each
        // relative to the reference pointclouds
        while (query_it != measurements[j].end())
        {
          // find reference pointcloud with timestamp closest to query pointcloud
          while (ref_it != measurements[i].end() 
            && ref_it->first < query_it->first)
          {
            ref_it++;
          }
          if (ref_it != measurements[i].begin() 
            && std::fabs((ref_it-1)->first - query_it->first) 
              < std::fabs(ref_it->first - query_it->first))
            ref_it--;

          // distances from each query point to nearest reference point
          std::vector<double> distances;

          if (std::fabs(ref_it->first - query_it->first) > 0.5)
            LOG(ERROR) << "time difference greater than 0.5s";
          
          for (size_t k = 0; k < query_it->second.getInputCloud()->size(); k++)
          {
            std::vector<int> indices;
            std::vector<float> sqr_dist;
            ref_it->second.nearestKSearch(query_it->second.getInputCloud()->at(k),
                                          1,
                                          indices,
                                          sqr_dist);
            if (sqr_dist.size() == 1)
            {
              distances.push_back(std::sqrt(sqr_dist[0]));
            }
            else if (sqr_dist.size() == 0)
              LOG(ERROR) << "no neighbors found";
            else
              LOG(FATAL) << "multiple neighbors found";
          }

          cloud_distances[i][j].push_back(std::make_pair(query_it->first, distances));

          query_it++;
        }
      }
    }
  }
}


// print statistics on each topic's agreement with the others
void getScores(std::vector<std::vector<distanceList>> &cloud_distances,
               std::vector<std::string> &topics)
{
  size_t num_topics = topics.size();

  for (size_t i = 0; i < num_topics; i++)
  {
    for (size_t j = 0; j < num_topics; j++)
    {
      if (i != j)
      {
        // collect all distances in one vector
        std::vector<double> distances;
        distanceList::const_iterator it = cloud_distances[i][j].begin();
        for (; it != cloud_distances[i][j].end(); it++)
        {
          distances.insert(distances.end(), it->second.begin(), it->second.end());
        }

        double mean_points = double(distances.size()) / double(cloud_distances[i][j].size());

        // sort
        std::sort(distances.begin(), distances.end());
        /*
        std::sort(distances.begin(), 
                  distances.end(), 
                  [](double lhs, double rhs)->bool{return lhs > rhs;});
        */
        // get statistics
        double min = distances.front();
        double max = distances.back();
        double median = distances.at(int(distances.size()/2));

        double sum = 0.0;
        int zero_count = 0;
        for (std::vector<double>::const_iterator it = distances.begin();
             it != distances.end(); it++)
        {
          sum += *it;
          if (*it < 0.01) zero_count++;
        }
        double mean = sum / double(distances.size());
        double mean_zero_count = double(zero_count) / double(cloud_distances[i][j].size());

        double sum_dev = 0.0;
        for (std::vector<double>::const_iterator it = distances.begin();
             it != distances.end(); it++)
          sum_dev += (*it - mean)*(*it - mean);
        double std_dev = std::sqrt(sum_dev / double(distances.size()));

        // generate report
        std::string report = "\n\n";
        report.append("Comparing map from " + topics[j] 
               + " to map from " + topics[i] + "\n");
        report.append("        mean points per scan: " 
          + std::to_string(mean_points) + "\n");
        report.append("mean zero distances per scan: " 
          + std::to_string(mean_zero_count) + "\n");
        report.append("                max distance: " 
          + std::to_string(max) + "\n");
        report.append("                min distance: " 
          + std::to_string(min) + "\n");
        report.append("             median distance: " 
          + std::to_string(median) + "\n");
        report.append("               mean distance: " 
          + std::to_string(mean) + "\n");
        report.append(" distance standard deviation: " 
          + std::to_string(std_dev) + "\n");
        report.append("\n\n");

        LOG(ERROR) << report;
      }
    }
  }
}

int main(int argc, char* argv[])
{
  google::InitGoogleLogging(argv[0]);

  std::string bagfile_name;
  std::vector<std::string> topics;

  if (argc > 2)
  {
    bagfile_name = std::string(argv[1]);
    for (int i = 2; i < argc; i++)
        topics.push_back(std::string(argv[i]));
  }
  else
  {
    LOG(FATAL) << "wrong number of arguments\n" 
               << "argument 1: <filename for rosbag> \n"
               << "arguments 2-n: list of octomap topics to compare (use octomap_point_cloud_centers)";
  }

  std::string name_prefix = bagfile_name.substr(0,bagfile_name.size()-4);
  std::string directory = name_prefix.substr(0,name_prefix.find_last_of('/')+1);

  size_t num_topics = topics.size();
  std::vector<std::vector<stampedMap>> measurements;
  measurements.resize(num_topics);

  LOG(ERROR) << "reading measurements from bag file";
  getMeasurements(measurements,
                  topics,
                  bagfile_name);
  
  std::vector<std::vector<distanceList>> cloud_distances;
  getDistanceLists(measurements, cloud_distances, topics);

  getScores(cloud_distances, topics);

  return 0;
}