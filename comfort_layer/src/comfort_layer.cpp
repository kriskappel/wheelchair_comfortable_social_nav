/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include "comfort_layer/comfort_layer.h"
#include <costmap_2d/costmap_math.h>
#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>

PLUGINLIB_EXPORT_CLASS(comfort_layer::ComfortLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;


namespace comfort_layer
{


void ComfortLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  ComfortLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  // std::string source;
  // while (ss >> source)
  // {
  //   ros::NodeHandle source_node(nh, source);

  //   // get the parameters for the specific topic
  //   double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
  //   std::string topic, sensor_frame, data_type;
  //   bool inf_is_valid, clearing, marking;

  //   source_node.param("topic", topic, source);
  //   source_node.param("sensor_frame", sensor_frame, std::string(""));
  //   source_node.param("observation_persistence", observation_keep_time, 0.0);
  //   source_node.param("expected_update_rate", expected_update_rate, 0.0);
  //   source_node.param("data_type", data_type, std::string("PointCloud"));
  //   source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
  //   source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
  //   source_node.param("inf_is_valid", inf_is_valid, false);
  //   source_node.param("clearing", clearing, false);
  //   source_node.param("marking", marking, true);

  //   if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
  //   {
  //     ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
  //     throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
  //   }

  //   std::string raytrace_range_param_name, obstacle_range_param_name;

  //   // get the obstacle range for the sensor
  //   double obstacle_range = 2.5;
  //   if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
  //   {
  //     source_node.getParam(obstacle_range_param_name, obstacle_range);
  //   }

  //   // get the raytrace range for the sensor
  //   double raytrace_range = 3.0;
  //   if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
  //   {
  //     source_node.getParam(raytrace_range_param_name, raytrace_range);
  //   }

  //   ROS_INFO("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
  //             sensor_frame.c_str());

  //   // create an observation buffer
  //   observation_buffers_.push_back(
  //       boost::shared_ptr < costmap_2d::ObservationBuffer
  //           > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
  //                                    max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
  //                                    sensor_frame, transform_tolerance)));

  //   // check if we'll add this buffer to our marking observation buffers
  //   if (marking)
  //     marking_buffers_.push_back(observation_buffers_.back());

  //   // check if we'll also add this buffer to our clearing observation buffers
  //   if (clearing)
  //     clearing_buffers_.push_back(observation_buffers_.back());

  //   ROS_INFO(
  //       "Created an observation buffer for source %s, topic %s, global frame: %s, "
  //       "expected update rate: %.2f, observation persistence: %.2f",
  //       source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

  //   // create a callback for the topic
  //   if (data_type == "LaserScan")
  //   {
  //     boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
  //         > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

  //     boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan> > filter(
  //       new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50, g_nh));

  //     if (inf_is_valid)
  //     {
  //       filter->registerCallback([this,buffer=observation_buffers_.back()](auto& msg){ laserScanValidInfCallback(msg, buffer); });
  //     }
  //     else
  //     {
  //       filter->registerCallback([this,buffer=observation_buffers_.back()](auto& msg){ laserScanCallback(msg, buffer); });
  //     }

  //     observation_subscribers_.push_back(sub);
  //     observation_notifiers_.push_back(filter);

  //     observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
  //   }
  //   else if (data_type == "PointCloud")
  //   {
  //     boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
  //         > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

  //     if (inf_is_valid)
  //     {
  //      ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
  //     }

  //       boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud>
  //       > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50, g_nh));
  //       filter->registerCallback([this,buffer=observation_buffers_.back()](auto& msg){ pointCloudCallback(msg, buffer); });

  //     observation_subscribers_.push_back(sub);
  //     observation_notifiers_.push_back(filter);
  //   }
  //   else
  //   {
  //     boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
  //         > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

  //     if (inf_is_valid)
  //     {
  //      ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
  //     }

  //     boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud2>
  //     > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50, g_nh));
  //     filter->registerCallback([this,buffer=observation_buffers_.back()](auto& msg){ pointCloud2Callback(msg, buffer); });

  //     observation_subscribers_.push_back(sub);
  //     observation_notifiers_.push_back(filter);
  //   }

  //   if (sensor_frame != "")
  //   {
  //     std::vector < std::string > target_frames;
  //     target_frames.push_back(global_frame_);
  //     target_frames.push_back(sensor_frame);
  //     observation_notifiers_.back()->setTargetFrames(target_frames);
  //   }
  // }

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);

  first_run_ = true;
  costmap_values_ = "";


  nh.param("activate_layer", active_layer_, true);
  nh.param("return_current_costmap_value", publish_costmap_value_, true);
  if(publish_costmap_value_)
    costmap_current_value_pub_ = nh.advertise<std_msgs::UInt8>("current_value", 1000);

  // std::ofstream MyFile("/home/kriskappel/costmap.txt");

}

void ComfortLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb =
      [this](auto& config, auto level){ reconfigureCB(config, level); };
  dsrv_->setCallback(cb);
}

ComfortLayer::~ComfortLayer()
{
    if (dsrv_)
        delete dsrv_;
}
void ComfortLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  combination_method_ = config.combination_method;
}

void ComfortLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer)
{
  // project the laser into a point cloud
  // sensor_msgs::PointCloud2 cloud;
  // cloud.header = message->header;

  // // project the scan into a point cloud
  // try
  // {
  //   projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  // }
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
  //            ex.what());
  //   projector_.projectLaser(*message, cloud);
  // }
  // catch (std::runtime_error &ex)
  // {
  //   ROS_WARN("transformLaserScanToPointCloud error, it seems the message from laser sensor is malformed. Ignore this laser scan. what(): %s", ex.what());
  //   return; //ignore this message
  // }

  // // buffer the point cloud
  // buffer->lock();
  // buffer->bufferCloud(cloud);
  // buffer->unlock();
}

void ComfortLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer)
{
  // Filter positive infinities ("Inf"s) to max_range.
  // float epsilon = 0.0001;  // a tenth of a millimeter
  // sensor_msgs::LaserScan message = *raw_message;
  // for (size_t i = 0; i < message.ranges.size(); i++)
  // {
  //   float range = message.ranges[ i ];
  //   if (!std::isfinite(range) && range > 0)
  //   {
  //     message.ranges[ i ] = message.range_max - epsilon;
  //   }
  // }

  // // project the laser into a point cloud
  // sensor_msgs::PointCloud2 cloud;
  // cloud.header = message.header;

  // // project the scan into a point cloud
  // try
  // {
  //   projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  // }
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
  //            global_frame_.c_str(), ex.what());
  //   projector_.projectLaser(message, cloud);
  // }
  // catch (std::runtime_error &ex)
  // {
  //   ROS_WARN("transformLaserScanToPointCloud error, it seems the message from laser sensor is malformed. Ignore this laser scan. what(): %s", ex.what());
  //   return; //ignore this message
  // }

  // // buffer the point cloud
  // buffer->lock();
  // buffer->bufferCloud(cloud);
  // buffer->unlock();
}

void ComfortLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                               const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer)
{
  sensor_msgs::PointCloud2 cloud2;

  if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
  {
    ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud2);
  buffer->unlock();
}

void ComfortLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void ComfortLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  // std::cout<<*max_y<<std::endl;
  // ROS_INFO("comfort %f %f %f %f %f %f", robot_x, robot_y, *min_x, *min_y, *max_x, *max_y);
  if (rolling_window_)
  {
    // ROS_WARN("rolling_window");
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  // ROS_WARN("The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.", robot_x, robot_y);
  

  // bool current = true;
  // std::vector<Observation> observations, clearing_observations;

  // // get the marking observations
  // current = current && getMarkingObservations(observations);

  // // get the clearing observations
  // current = current && getClearingObservations(clearing_observations);

  // // update the global current status
  // current_ = current;

  // raytrace freespace
  // for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  // {
  //   raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  // }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  // for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  // {
  //   const Observation& obs = *it;

  //   const sensor_msgs::PointCloud2& cloud = *(obs.cloud_);

  //   double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

  //   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  //   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  //   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  //   for (; iter_x !=iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  //   {
  //     double px = *iter_x, py = *iter_y, pz = *iter_z;

  //     // if the obstacle is too high or too far away from the robot we won't add it
  //     if (pz > max_obstacle_height_)
  //     {
  //       ROS_DEBUG("The point is too high");
  //       continue;
  //     }

  //     // compute the squared distance from the hitpoint to the pointcloud's origin
  //     double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
  //         + (pz - obs.origin_.z) * (pz - obs.origin_.z);

  //     // if the point is far enough away... we won't consider it
  //     if (sq_dist >= sq_obstacle_range)
  //     {
  //       ROS_DEBUG("The point is too far away");
  //       continue;
  //     }

  //     // now we need to compute the map coordinates for the observation
  //     unsigned int mx, my;
  //     if (!worldToMap(px, py, mx, my))
  //     {
  //       ROS_DEBUG("Computing map coords failed");
  //       continue;
  //     }


  //     // MyFile << std::to_string(mx) + " " + std::to_string(my) + " ";
  //     // ROS_INFO("%d %d", mx, my);

  //     unsigned int index = getIndex(mx, my);
  //     costmap_[index] = LETHAL_OBSTACLE;
  //     touch(px, py, min_x, min_y, max_x, max_y);
  //   }
  // }

  worldToMap(robot_x, robot_y, rx_, ry_);
  // ROS_INFO("comfort layer robot x y %f %f %u %u", robot_x, robot_y, rx_, ry_);
  // ROS_INFO("comfort layer cells size x y %u %u origin x y %f %f", getSizeInCellsX(), getSizeInCellsY(), getOriginX(), getOriginY());

  // if(publish_costmap_value_)
  // {
  //   std_msgs::UInt8 robot_cell_cost;;
  //   robot_cell_cost.data = static_cast<uint8_t>(costmap_[getIndex(rx_, ry_)]); //convert uint to uchar
    
  //   costmap_current_value_pub_.publish(robot_cell_cost);
  // }
  // if (first_run_ == 1)
  // {
  //   // printCostmap(*min_x, *min_y, *max_x, *max_y);
  //   // first_run_ = 0;
  // }

  // if (*max_y < 1000)
  // {
  //   
  //   first_run_ = 1;
  // }
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ComfortLayer::printCostmap(double min_x, double min_y, double max_x, double max_y)
{

  // std::ofstream MyFile("/home/kriskappel/costmap.txt");

  // for(unsigned int j=min_y; j<min_y; ++j)
  // {
  //   for(unsigned int i=min_x; i<min_x; ++i)
  //   {
  //     unsigned int index = getIndex(i, j);

  //     MyFile << std::to_string(costmap_[index]) + " ";
  //     // costmap_values_ = costmap_values_ +  std::to_string(costmap_[index]) + " ";
  //   }
  //   // costmap_values_ = costmap_values_ + "\n";
  //   MyFile << "\n";

  // }

  // // MyFile << costmap_values_;
  // MyFile.close();
  // ROS_INFO("%lf %lf %lf %lf", min_x, min_y, max_x, max_y);
  // std::cout<< min_x << " " << min_y << " " << max_x << " " << max_y;
}

void ComfortLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    // if (!footprint_clearing_enabled_) return;
    // costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    // for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    // {
    //   touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    // }
}

void ComfortLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  // if (footprint_clearing_enabled_)
  //   setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);

  // ROS_INFO("%d %d %d %d", min_i, min_j, max_i, max_j);

  unsigned char * master_array = master_grid.getCharMap();

  std::vector<std::pair<cv::Vec4i, cv::Vec4i>> parallelPairs;
  // // if (first_run_ == 0)
  // // {
  // //   // first_run_ = 0;

  // //   // std::ofstream MyFile("/home/kriskappel/costmap.txt");
  // //   // ROS_INFO("%d %d %d %d", min_i, min_j, max_i, max_j);
  // // //   ROS_INFO("Teste");
  // std::cout<<"\ncostmap before comfort\n";
  // for(int j = min_j; j < max_j; j++) 
  // {
  //   for (int i = min_i; i < max_i; i++) 
  //   { 
  //     unsigned int index = getIndex(i, j);
  //     // inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  //     // {
  //     //   return my * size_x_ + mx;
  //     // }
  //     // MyFile << costmap_[index];
  //     // MyFile << " ";
      
  //     // ROS_INFO("%d %d %d", i, j, costmap_[index]);  
  //     // std::cout << (int)master_array[index] << " ";
  //   }// 
  //   /// / MyFile << "\n";
  //   std::cout<<"\n";
  // }
 //  // // }
  // // first_run_ = 0;

  if(first_run_)
  {
    ROS_INFO("FIRST RUN");
    first_run_ = false;

    height_ = (max_j - min_j);
    width_ = (max_i - min_i);
   
    // unsigned char * saved_map_ = new unsigned char[height * width];


    translate_array_ = new unsigned int[(max_i - min_i) * (max_j - min_j)];
    parallelPairs = findParallelLines(master_array,  min_i,  min_j,  max_i,  max_j);
    //std::cout<<parallelPairs.size()<<std::endl;
    // for (int i = 0; i < (max_i - min_i) * (max_j - min_j); ++i)
    // {
    //   std::cout<<translate_array_[i]<< " ";
    // }
    // std::cout<<std::endl;
    std::vector<costmap_2d::MapLocation> polygon_cells;
    
    std::vector<costmap_2d::MapLocation> map_polygon;

    int pairsCount = 0;
    while (!parallelPairs.empty())
    {
      std::pair<cv::Vec4i, cv::Vec4i> pp = parallelPairs.back();
      parallelPairs.pop_back();
      // pair = parallelPairs[0];
      cv::Vec4i line1 = pp.first;
      cv::Vec4i line2 = pp.second;

      // std::cout << "Line 1: (" << line1[0] << ", " << line1[1] << ") - (" << line1[2] << ", " << line1[3] << ")" << std::endl;
      // std::cout << "Line 2: (" << line2[0] << ", " << line2[1] << ") - (" << line2[2] << ", " << line2[3] << ")" << std::endl;

      costmap_2d::MapLocation maploc;


      //true case lines are verticals, false otherwise. 
      //need only to test one line case both are vertical or horizontal
      if(abs(line1[1] - line1[3]) < abs(line1[0] - line1[2])) 
      {

         // ROS_INFO("vertical");
        maploc.x = line1[0] + min_i;
        maploc.y = line1[1] + min_j;  
        map_polygon.push_back(maploc);

        // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";

        maploc.x = line1[2] + min_i;
        maploc.y = line1[3] + min_j;  
        map_polygon.push_back(maploc);
        // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";

        maploc.x = line2[0] + min_i;
        maploc.y = line2[1] + min_j;  
        map_polygon.push_back(maploc);
        // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";

        maploc.x = line2[2] + min_i;
        maploc.y = line2[3] + min_j;  
        map_polygon.push_back(maploc);
        // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";
      }
      else
      {
         // ROS_INFO("horizontal");
        maploc.x = line1[0] + min_i;
        maploc.y = line1[1] + min_j;  
        map_polygon.push_back(maploc);

        // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";

        maploc.x = line2[0] + min_i;
        maploc.y = line2[1] + min_j;  
        map_polygon.push_back(maploc);
        // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";

        maploc.x = line1[2] + min_i;
        maploc.y = line1[3] + min_j;  
        map_polygon.push_back(maploc);
        // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";


        maploc.x = line2[2] + min_i;
        maploc.y = line2[3] + min_j;  
        map_polygon.push_back(maploc);
        // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";
      }
      

      convexFillCells(map_polygon, polygon_cells);

      for (unsigned int i = 0; i < polygon_cells.size(); ++i)
      {
        unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
        unsigned int  print_map_x;
        unsigned int  print_map_y; 
        indexToCells(index, print_map_x, print_map_y);
        double print_world_x;
        double print_world_y;
        mapToWorld(print_map_x, print_map_y, print_world_x, print_world_y);
        

        double pos_coor = positionInCorridor(cv::Point(line1[0] + min_i, line1[1]+ min_j), cv::Point(line1[2]+ min_i, line1[3]+ min_j), 
          cv::Point(line2[0]+ min_i, line2[1]+ min_j), cv::Point(line2[2]+ min_i, line2[3]+ min_j)  , polygon_cells[i].x, polygon_cells[i].y);  

        double comfort = calculateComfortFunction(pos_coor);

        if (master_array[index] < LETHAL_OBSTACLE)
        {
          master_array[index] = mapComfortToCost(comfort);

          // saved_map_[index] = mapComfortToCost(comfort);
          // unsigned int  print_map_x;
          // unsigned int  print_map_y; 
          // indexToCells(index, print_map_x, print_map_y);
          // double print_world_x;
          // double print_world_y;
          // mapToWorld(print_map_x, print_map_y, print_world_x, print_world_y);
          // ROS_INFO("comfort func %d positionInCorridor %f comfort %f x %f y %f", mapComfortToCost(comfort), pos_coor, comfort, print_world_x, print_world_y);
          // ROS_INFO("comfort func %d positionInCorridor %f comfort %f", mapComfortToCost(comfort), pos_coor, comfort);
          // if(mapComfortToCost(comfort) > )
          // std::cout <<(int)master_array[index]<< " ";
        }
        // costmap_[index] = 200;
        // ;
      }
      saveMapCopy(master_array, height_ * width_);
      map_polygon.clear();
      polygon_cells.clear();
      // for (int j = 0; j < 384; j++) 
      // {
      //   for (int i = 0; i < 384; i++) 
      //   {
      //     // master_array[getIndex(i,j)] = saved_map_[getIndex(i,j)];
      //     // master_array[getIndex(i,j)] = ;
      //     ROS_INFO("%d",  saved_map_[getIndex(i, j)]);
      //     // std::cout<<saved_map_[getIndex(i,j)];
          
      //   }
        
      // }
      // ROS_INFO("ending");
    }
    
  }

  if(!first_run_)
  {
    // ROS_INFO("hello?");
    // for (int j = 0; j < 384; j++) 
    // {
    //   for (int i = 0; i < 384; i++) 
    //   {
    //     master_array[getIndex(i,j)] = saved_map_[getIndex(i,j)];
    //     // master_array[getIndex(i,j)] = ;
    //     // ROS_INFO("%d",  saved_map_[getIndex(i, j)]);
    //     // std::cout<<saved_map_[getIndex(i,j)];
        
    //   }
    // }
    if(publish_costmap_value_)
    {
      std_msgs::UInt8 robot_cell_cost;;
      robot_cell_cost.data = static_cast<uint8_t>(saved_map_[getIndex(rx_, ry_)]); //convert uint to uchar
      
      costmap_current_value_pub_.publish(robot_cell_cost);
    }

    if(active_layer_)
      std::memcpy(master_array, saved_map_, height_ * width_);
  }  

  // unsigned int ix, jx;
  // ix = 55;
  // jx = 294;
  // for (int i = 0; i < 20; ++i)
  // {
  //   // ROS_INFO("%d",  master_array[getIndex(ix, jx) + i]);
  //   master_array[getIndex(ix, jx) + i]= master_array[getIndex(ix, jx) + i] +  1;
  // }
    // for (int j = min_j; j < max_j; j++) 
  // {
  //     for (int i = min_i; i < max_i; i++) 
  //     {
  //       ROS_INFO("%d %d %d",  i, j, master_array[getI ndex(i,j)]);
  //       // unsigned int index = getIndex(i, j);
  //       // master_array[index] = saved_map_[index];
  //       // updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  //     }
  //   }
  // }
  // else
  // {
  //   for (int j = min_j; j < max_j; j++) 
  //   
  //     for (int i = min_i; i < max_i; i++) 
  //     {
  //       unsigned int index = getIndex(i, j);
  //       master_array[index] = saved_map_[index];
  //       updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  //     }
  //   }
  // }
  // std::cout << "--------------------------" << std::endl;

  // std::cout<<"\ncostmap afer comfort\n";
  // for (int j = min_j; j < max_j; j++) 
  // {
  //   for (int i = min_i; i < max_i; i++) 
  //   {
  //     unsigned int index = getIndex(i, j);
  //     // inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  //     // {
  //     //   return my * size_x_ + mx;
  //     // }
  //     // MyFile << costmap_[index];
  //     // MyFile << " ";
  //     // if (costmap_[index] == LETHAL_OBSTACLE)
  //     // ROS_INFO("%d %d %d", i, j, costmap_[index]);  
  //     std::cout << (int)master_array[index] << " ";
  //   }
  //   // MyFile << "\n";
  //   std::cout<<"\n";
  // }

  
  // switch (combination_method_)
  // {
  //   case 0:  // Overwrite
  //     updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  //     break;
  //   case 1:  // Maximum
  //     updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  //     break;
  //   default:  // Nothing
  //     break;
  // }

  // if (!parallelPairs.empty())
  // {
  //   for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  //   {
  //     unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);

  //     if (costmap_[index] != LETHAL_OBSTACLE)
  //       costmap_[index] = 0; 
  //   }
  // }


  // for (const auto& pair : parallelPairs) 
  // {
  //   // pair = parallelPairs[0];
  //   cv::Vec4i line1 = pair.first;
  //   cv::Vec4i line2 = pair.second;

  //   // std::cout << "Line 1: (" << line1[0] << ", " << line1[1] << ") - (" << line1[2] << ", " << line1[3] << ")" << std::endl;
  //   // std::cout << "Line 2: (" << line2[0] << ", " << line2[1] << ") - (" << line2[2] << ", " << line2[3] << ")" << std::endl;
    
  //   costmap_2d::MapLocation maploc;

  //   maploc.x = line1[0] + min_i;
  //   maploc.y = line1[1] + min_j;  
  //   map_polygon.push_back(maploc);

  //   // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";

  //   maploc.x = line1[2] + min_i;
  //   maploc.y = line1[3] + min_j;  
  //   map_polygon.push_back(maploc);
  //   // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";

  //   maploc.x = line2[0] + min_i;
  //   maploc.y = line2[1] + min_j;  
  //   map_polygon.push_back(maploc);
  //   // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";

  //   maploc.x = line2[2] + min_i;
  //   maploc.y = line2[3] + min_j;  
  //   map_polygon.push_back(maploc);
  //   // std::cout << "maploc " <<maploc.x<< " " <<maploc.y<<"\n";


  //   std::vector<costmap_2d::MapLocation> polygon_cells;

  //   convexFillCells(map_polygon, polygon_cells);

  //   for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  //   {
  //     unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);

  //     double pos_coor = positionInCorridor(cv::Point(line1[0], line1[1]), cv::Point(line1[2], line1[3]), 
  //       cv::Point(line2[0], line2[1]), cv::Point(line2[2], line2[3]), polygon_cells[i].x, polygon_cells[i].y);  

  //     double comfort = calculateComfortFunction(pos_coor);

  //     if (costmap_[index] != LETHAL_OBSTACLE)
  //       // costmap_[index] = mapComfortToCost(comfort);
  //       costmap_[index] = 0; 
  //     // ROS_INFO("comfort %d ", mapComfortToCost(comfort));
  //   }
  //   // map_polygon.clear();

  // }
}

void ComfortLayer::saveMapCopy(unsigned char* original_map, size_t size) {
  size_map_ = size;
  saved_map_ = new unsigned char[size_map_];
  std::memcpy(saved_map_, original_map, size_map_);
}


void ComfortLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

void ComfortLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

bool ComfortLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  // // get the marking observations
  // for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  // {
  //   marking_buffers_[i]->lock();
  //   marking_buffers_[i]->getObservations(marking_observations);
  //   current = marking_buffers_[i]->isCurrent() && current;
  //   marking_buffers_[i]->unlock();
  // }
  // marking_observations.insert(marking_observations.end(),
  //                             static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool ComfortLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // // get the clearing observations
  // for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  // {
  //   clearing_buffers_[i]->lock();
  //   clearing_buffers_[i]->getObservations(clearing_observations);
  //   current = clearing_buffers_[i]->isCurrent() && current;
  //   clearing_buffers_[i]->unlock();
  // }
  // clearing_observations.insert(clearing_observations.end(),
  //                             static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void ComfortLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{
  // double ox = clearing_observation.origin_.x;
  // double oy = clearing_observation.origin_.y;
  // const sensor_msgs::PointCloud2 &cloud = *(clearing_observation.cloud_);

  // // get the map coordinates of the origin of the sensor
  // unsigned int x0, y0;
  // if (!worldToMap(ox, oy, x0, y0))
  // {
  //   ROS_WARN_THROTTLE(
  //       1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
  //       ox, oy);
  //   return;
  // }

  // // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  // double origin_x = origin_x_, origin_y = origin_y_;
  // double map_end_x = origin_x + size_x_ * resolution_;
  // double map_end_y = origin_y + size_y_ * resolution_;


  // touch(ox, oy, min_x, min_y, max_x, max_y);

  // // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  // sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  // sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  // for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
  // {
  //   double wx = *iter_x;
  //   double wy = *iter_y;

  //   // now we also need to make sure that the enpoint we're raytracing
  //   // to isn't off the costmap and scale if necessary
  //   double a = wx - ox;
  //   double b = wy - oy;

  //   // the minimum value to raytrace from is the origin
  //   if (wx < origin_x)
  //   {
  //     double t = (origin_x - ox) / a;
  //     wx = origin_x;
  //     wy = oy + b * t;
  //   }
  //   if (wy < origin_y)
  //   {
  //     double t = (origin_y - oy) / b;
  //     wx = ox + a * t;
  //     wy = origin_y;
  //   }

  //   // the maximum value to raytrace to is the end of the map
  //   if (wx > map_end_x)
  //   {
  //     double t = (map_end_x - ox) / a;
  //     wx = map_end_x - .001;
  //     wy = oy + b * t;
  //   }
  //   if (wy > map_end_y)
  //   {
  //     double t = (map_end_y - oy) / b;
  //     wx = ox + a * t;
  //     wy = map_end_y - .001;
  //   }

  //   // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
  //   unsigned int x1, y1;

  //   // check for legality just in case
  //   if (!worldToMap(wx, wy, x1, y1))
  //     continue;

  //   unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
  //   MarkCell marker(costmap_, FREE_SPACE);
  //   // and finally... we can execute our trace to clear obstacles along that line
  //   raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

  //   updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  // }
}

void ComfortLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
}
void ComfortLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
}

void ComfortLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ComfortLayer::reset()
{
    deactivate();
    resetMaps();
    current_ = true;
    activate();
}

// Function to calculate the distance between two points
double ComfortLayer::calculateDistancePointPoint(const cv::Point& p1, const cv::Point& p2) 
{
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;

    return sqrt(dx * dx + dy * dy);
}

double ComfortLayer::calculateDistancePointLine(double x, double y, double x1, double y1, double x2, double y2) 
{
    double dx = x2 - x1;
    double dy = y2 - y1;

    double numerator = std::abs(dy * x - dx * y + x2 * y1 - x1 * y2);
    double denominator = std::sqrt(dx * dx + dy * dy);

    return numerator / denominator;
}

double ComfortLayer::positionInCorridor(const cv::Point& p1_l1, const cv::Point& p2_l1, const cv::Point& p1_l2, const cv::Point& p2_l2, double x, double y)
{
    double distance1 = calculateDistancePointLine(x, y, p1_l1.x, p1_l1.y, p2_l1.x, p2_l1.y);
    double distance2 = calculateDistancePointLine(x, y, p1_l2.x, p1_l2.y, p2_l2.x, p2_l2.y);
    
    // double minDistance = (distance1 < distance2) ? distance1 : distance2; //getting min distance between distance 1 or 2 using ternary if.
    double minDistance;
    if (distance1 < distance2)
      minDistance = distance1;
    else 
      minDistance = distance2;
    

    return minDistance/(distance1 + distance2);
}

double ComfortLayer::calculateComfortFunction(double y)
{
    double a = 0.1;
    double b = 0.3;
    double c = 0.25;

    return std::exp(-1 * ( (a/y) + (a/(1-y)) + pow( ((y-c) / b), 2) ) );

}

unsigned char ComfortLayer::mapComfortToCost(double value) 
{
  // ROS_INFO("value %f ", value);
  // Check if the value is within the valid range
  if (value < 0 || value > 0.605) {
    // Handle out-of-range values
    // You can customize the behavior here, like throwing an exception or returning a default value
    ROS_INFO("value %f ", value);
    return -1; // For example, returning -1 as an error indicator
  }

  // Calculate the corresponding int value
  // ROS_INFO("%f ", (1 - value / 0.605) * 254);
  unsigned char result = static_cast<unsigned char>((1 - value / 0.605) * 252);
  // if (result > 254)
  //   result = 254;
  if ((int) result > 252)
    ROS_INFO("value %d ", result);
  return result;
}

bool compareLineLength(const cv::Vec4i& line1, const cv::Vec4i& line2) {
    // Calculate the squared length of each line
    int length1 = (line1[2] - line1[0]) * (line1[2] - line1[0]) + (line1[3] - line1[1]) * (line1[3] - line1[1]);
    int length2 = (line2[2] - line2[0]) * (line2[2] - line2[0]) + (line2[3] - line2[1]) * (line2[3] - line2[1]);

    return length1 > length2; // Sort in descending order of line length
}

std::vector<cv::Vec4i> ComfortLayer::sortLinesByLength(const std::vector<cv::Vec4i>& lines) {
    std::vector<cv::Vec4i> sortedLines(lines);
    std::sort(sortedLines.begin(), sortedLines.end(), compareLineLength);
    return sortedLines;
}


// Function to find parallel lines in a binary image
std::vector<std::pair<cv::Vec4i, cv::Vec4i>> ComfortLayer::findParallelLines(unsigned char* data, int min_i, int min_j, int max_i, int max_j) {
    // ROS_INFO("findParallelLines func");
    int height = (max_j - min_j);
    int width = (max_i - min_i);
    int l = 0;
    int k = 0;
    // std::cout<< sizeof(new unsigned char) << " " << sizeof(unsigned char[height * width]);
    unsigned char * costmap_aux = new unsigned char[height * width];

    

    for (int j = min_j; j < max_j; j++, l++) 
    {
      
      for (int i = min_i; i < max_i; i++, k++) 
      {
        int index = getIndex(i, j);
        
        int index_aux = k+l * width;

        translate_array_[index_aux] = index;

        // if(height < 300)
        // {
        //   std::cout<<i<<" "<< j<<" "<< index <<std::endl;
        //   std::cout<<k<<" "<< l<<" "<< index_aux <<std::endl;
        // }
        if(data[index] != LETHAL_OBSTACLE)
          costmap_aux[index_aux] = 0;
        else
          costmap_aux[index_aux] = 255;

        // ROS_INFO("%d", costmap_aux[index_aux]);
        // std::cout << (int)costmap_aux[index_aux] << " ";

      }
      // std::cout<<std::endl;
      // l=0;
      k=0;
    } //TODO Preciso achar um jeito de 
    // ROS_INFO("testing");
    // Convert data to binary image
    cv::Mat binaryImage(height, width, CV_8UC1, costmap_aux);
    

    double angleThreshold = CV_PI / 36; // 5 degrees threshold for diagonal lines
    double minDistanceThreshold = 3.0;   //3.0; // Set your desired threshold value here
    double maxDistanceThreshold = 100.0; //50.0;
    double inclinationThreshold = 6.0;   //6.0;
    int maxLineGap = 6; //2
    int minLineLength = 15; //15
    int minNumPointsToLine = 25; //25
    // ROS_INFO("testing2");
    // cv::Mat thresholdImage;
    // cv::threshold(binaryImage, thresholdImage, 0, 255, cv::THRESH_BINARY);
    // std::cout << binaryImage << std::endl << std::endl;
    // ROS_INFO("findParallelLines %d %d %d %d", min_i, min_j, max_i, max_j);

    // Apply Hough Transform to detect lines
    std::vector<cv::Vec4i> lines;
    // cv::HoughLinesP(binaryImage, lines, 1, CV_PI / 180, 25, 15, 2);
    cv::HoughLinesP(binaryImage, lines, 1, CV_PI / 180, minNumPointsToLine, minLineLength, maxLineGap);
     
    lines = sortLinesByLength(lines);
    // // Iterate over the lines to find parallel pairs  
    
    std::vector<std::pair<cv::Vec4i, cv::Vec4i>> parallelPairs;
    // std::cout<<lines.size()<<std::endl;
    for (size_t i = 0; i < lines.size(); i++) 
    {

        cv::Point pt1_1 = cv::Point(lines[i][0], lines[i][1]);
        cv::Point pt2_1 = cv::Point(lines[i][2], lines[i][3]);
        // Calculate the angle difference between lines
        double angle1 = std::atan2(pt2_1.y - pt1_1.y, pt2_1.x - pt1_1.x);
        double angle1degrees = angle1 * 180 / CV_PI;
        // ROS_INFO("angle1 %f \n", angle1);
        // ROS_INFO("pt1_1 %d %d \n", lines[i][0], lines[i][1]);
        // ROS_INFO("pt2_1 %d %d \n", lines[i][2], lines[i][3]);
        // Horizontal line (angle approximately 0 degree) and vertical line (angle approximately 90 degree)
        // if ( (abs(angle1degrees) < 10 || abs(angle1degrees - 180) < 10) || (abs(angle1degrees - 90) < 10 || abs(angle1degrees + 90) < 10) )
        // {   

          for (size_t j = i + 1; j < lines.size(); j++) 
          {
              cv::Point pt1_2 = cv::Point(lines[j][0], lines[j][1]);
              cv::Point pt2_2 = cv::Point(lines[j][2], lines[j][3]);

              // Calculate the angle difference between lines
              double angle2 = std::atan2(pt2_2.y - pt1_2.y, pt2_2.x - pt1_2.x);
              double angle2degrees = angle2 * 180 / CV_PI;
              double angleDiff = std::abs(angle1 - angle2);

              int diff1x = std::abs(pt2_1.x - pt1_1.x);
              int diff1y = std::abs(pt2_1.y - pt1_1.y);
              int diff2x = std::abs(pt2_2.x - pt1_2.x);
              int diff2y = std::abs(pt2_2.y - pt1_2.y);

              // Horizontal line (angle approximately 0 degree) and vertical line (angle approximately 90 degree)
              // if ( (abs(angle2degrees) < 10 || abs(angle2degrees - 180) < 10) || (abs(angle2degrees - 90) < 10 || abs(angle2degrees + 90) < 10) )
              // {

                // if(parallelPairs.size() > 4)
                //   return parallelPairs;
                //diff1x e diff2x é a diferença dos dois pontos da mesma reta em x
                //diff1y e diff2y é a diferença dos dois pontos da mesma reta em y
                if((diff1x < inclinationThreshold  && diff2x < inclinationThreshold ) || (diff1y < inclinationThreshold && diff2y < inclinationThreshold )) 
                {

                  // if (angleDiff > angleThreshold && angleDiff < (CV_PI - angleThreshold)) {
                  //maybe the negative part is not needed once angleDiff is the result of abs
                  if (angleDiff < angleThreshold && angleDiff > -1 * angleThreshold) 
                  {
                      
                      // Calculate the distance between the parallel lines
                      double distance = calculateDistancePointPoint((pt1_1 + pt2_1) * 0.5, (pt1_2 + pt2_2) * 0.5);
                      

                      //if true the lines are vertical, else the lines are horizontal
                      if(std::abs(pt1_1.x - pt2_1.x) < std::abs(pt1_1.y - pt2_1.y))
                      {
                        // Check if the distance is within the threshold
                        if(std::abs(pt1_1.x - pt1_2.x) < maxDistanceThreshold && std::abs(pt1_1.x - pt1_2.x) > minDistanceThreshold)
                        {
                          
                          //sets both the lines to the size of the smallest line
                          if(std::abs(lines[i][1]-lines[i][3]) < std::abs(lines[j][1]-lines[j][3]))
                          {
                            lines[j][1] = lines[i][1];
                            lines[j][3] = lines[i][3];
                          }
                          else
                          {
                            lines[i][1] = lines[j][1];
                            lines[i][3] = lines[j][3];
                          }

                          parallelPairs.emplace_back(lines[i], lines[j]);
                          // ROS_INFO("distance %f \n", distance);
                          // delete costmap_aux;
                          
                          // return parallelPairs;
                        }
                      }
                      else
                      {
                        // Check if the distance is within the threshold
                        if(std::abs(pt1_1.y - pt1_2.y) < maxDistanceThreshold && std::abs(pt1_1.y - pt1_2.y) > minDistanceThreshold)
                        {
                          //sets both the lines to the size of the smallest line
                          if(std::abs(lines[i][0]-lines[i][2]) < std::abs(lines[j][0]-lines[j][2]))
                          {
                            lines[j][0] = lines[i][0];
                            lines[j][2] = lines[i][2];
                          }
                          else
                          {
                            lines[i][0] = lines[j][0];
                            lines[i][2] = lines[j][2];
                          }

                          parallelPairs.emplace_back(lines[i], lines[j]);
                          // ROS_INFO("distance %f \n", distance);
                          // delete costmap_aux;

                          // return parallelPairs;
                        }
                      }
                  }
                    // if(std::abs(pt1_1.x - pt1_2.x) < maxDistanceThreshold && std::abs(pt1_1.x - pt1_2.x) > minDistanceThreshold
                    //   && std::abs(pt1_1.y - pt1_2.y) < maxDistanceThreshold && std::abs(pt1_1.y - pt1_2.y) > minDistanceThreshold)
                    // {
                    // // if (distance < maxDistanceThreshold && distance > minDistanceThreshold) {
                        
                    //     // std::cout<<"distance "<<distance<< " ";
                    // }
                }
              // }
          }
        // }
    }
    // std::cout<<"-----";

    // delete costmap_aux;

    return parallelPairs;
}






}  // namespace comfort_layer