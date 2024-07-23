/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
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
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "vision_layer/vision_layer.hpp"
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/costmap_math.h>

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;



PLUGINLIB_EXPORT_CLASS(vision_layer::VisionLayer, costmap_2d::Layer)


namespace vision_layer
{

  // VisionLayer::VisionLayer(): 
  //   last_min_x_(-std::numeric_limits<float>::max()),
  //   last_min_y_(-std::numeric_limits<float>::max()),
  //   last_max_x_(std::numeric_limits<float>::max()),
  //   last_max_y_(std::numeric_limits<float>::max())
  // {
  //   costmap_=NULL;
  // }

    // This method is called at the end of plugin initialization.
    // It contains ROS parameter(s) declaration and initialization
    // of need_recalculation_ variable.
    void VisionLayer::onInitialize()
    {
      ros::NodeHandle nh("~/" + name_);

      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
      dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&VisionLayer::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);

      need_recalculation_ = false;
      current_ = true;

      rolling_window_ = layered_costmap_->isRolling();
      //std::cout<<"rolling window"<<rolling_window_<<std::endl;

      VisionLayer::matchSize();

      nh.param("activate_layer", active_layer_, true);
      nh.param("return_current_costmap_value", publish_costmap_value_, true);
      if(publish_costmap_value_)
        costmap_current_value_pub_ = nh.advertise<std_msgs::UInt8>("current_value", 1000);

    } 


    void VisionLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
      enabled_ = config.enabled;
    }
    // The method is called to ask the plugin: which area of costmap it needs to update.
    // Inside this method window bounds are re-calculated if need_recalculation_ is true
    // and updated independently on its value.
    void VisionLayer::updateBounds( double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y)
    {



      if (rolling_window_)
      {
        //ROS_WARN("rolling_window");
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
      }
      // updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
      // ROS_INFO("vision updateBounds");
      // ROS_INFO("vision layer robot x y %f %f %f %f", robot_x, robot_y, origin_x_, origin_y_);
      if (need_recalculation_) 
      {
        // ROS_WARN("Recalculation needed");
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        // For some reason when I make these -<double>::max() it does not
        // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
        // -<float>::max() instead.
        *min_x = -std::numeric_limits<float>::max();
        *min_y = -std::numeric_limits<float>::max();
        *max_x = std::numeric_limits<float>::max();
        *max_y = std::numeric_limits<float>::max();
        need_recalculation_ = false;
      } 
      else 
      {
        // ROS_WARN("No Recalculation needed");
        double tmp_min_x = last_min_x_;
        double tmp_min_y = last_min_y_;
        double tmp_max_x = last_max_x_;
        double tmp_max_y = last_max_y_;
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        *min_x = std::min(tmp_min_x, *min_x);
        *min_y = std::min(tmp_min_y, *min_y);
        *max_x = std::max(tmp_max_x, *max_x);
        *max_y = std::max(tmp_max_y, *max_y);
      }

      // ROS_INFO("robot pos %f %f", robot_x, robot_y);

      bool bool_worldToMap = worldToMap(robot_x, robot_y, rx_, ry_);
      flag_raytrace_ = true;
      if(!bool_worldToMap)
      {
        //flag_raytrace_ = false;
        ROS_WARN("The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.", robot_x, robot_y);
        ROS_INFO("vision %f %f %u %u", robot_x, robot_y, rx_, ry_);
        // return;
      }
      if(publish_costmap_value_)
      {
        
        std_msgs::UInt8 robot_cell_cost;
        //ROS_INFO("pub costmap value");
        // unsigned int r_index = getIndex(rx_, ry_);
        robot_cell_cost.data = static_cast<uint8_t>(costmap_[getIndex(rx_, ry_)]); //convert uint to uchar
        
        costmap_current_value_pub_.publish(robot_cell_cost);
        //std::cout<<robot_cell_cost.data<<std::endl;

      }

      
      // ROS_INFO("%d %d", robot_x_, robot_y_);


    }

    
    // The method is called when footprint was changed.
    // Here it just resets need_recalculation_ variable.
    void VisionLayer::onFootprintChanged()
    {
      need_recalculation_ = true;

      ROS_DEBUG("VisionLayer::onFootprintChanged(): num footprint points: %lu", layered_costmap_->getFootprint().size());
    }

    // The method is called when costmap recalculation is required.
    // It updates the costmap within its window bounds.
    // Inside this method the costmap gradient is generated and is writing directly
    // to the resulting costmap master_grid without any merging with previous layers.
    void VisionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,int max_i,int max_j)
    {




      // if((max_i - min_i) >300)
      //   return;

      //ROS_INFO("vision updateCosts");
      //ROS_INFO("%d %d %d %d", min_i, min_j, max_i, max_j);
      // ROS_INFO("robot x and y, %d %d %d", rx_, ry_, flag_raytrace_);

      if(!flag_raytrace_)
        return;

      int i, j, k, l, index, index2, index3, local_i, local_j;
      unsigned char cost;
      
      unsigned int visibility_window_size = 60;

      int cell_raytrace_range = 15;

      cost = 200;

      unsigned int visible_cells = 0;
      unsigned int invisible_cells = 0;

      unsigned char * master_array = master_grid.getCharMap();

      float result;

      // ROS_INFO("LOCAL MAP");

      // for (int t = min_j; t < max_j; t++) 
      // {
      //   for (int f = min_i; f < max_i; f++) 
      //   {
      //     int index2=getIndex(f, t);
      //     std::cout<<(int)local_map[index2]<<" ";
      //   }
      //   std::cout<<std::endl;
      // }

      // ROS_INFO("MASTER ARRAY");

      // for (int t = min_j; t < max_j; t++) 
      // {
      //   for (int f = min_i; f < max_i; f++) 
      //   {
      //     int index2=getIndex(f, t);
      //     std::cout<<(int)master_array[index2]<<" ";
      //   }
      //   std::cout<<std::endl;
      // }

      


      //VisionCell vc(master_array, cost, visible_cells, invisible_cells);


      //MarkCell marker(master_array, cost);

      for (j = min_j; j < max_j; j++) 
      {
        
        for (i = min_i; i < max_i; i++) 
        {
          index = getIndex(i, j);
          //ROS_INFO("index %d", index);
          //ROS_INFO("checkpoint");
          //ROS_INFO("costmap index %u", costmap_[index]);
          if(master_array[index] == LETHAL_OBSTACLE)
          {
            //ROS_INFO("LETHAL_OBSTACLE");
            costmap_[index] = LETHAL_OBSTACLE;
            
            continue;
          }

          else if (master_array[index] == NO_INFORMATION)
          {
            //ROS_INFO("NO_INFORMATION");
            costmap_[index] = NO_INFORMATION;
            
            continue;
          }
          //ROS_INFO("debug");
          if(rolling_window_)
          {
            result = computeLocalVisibilityIndex(master_array, min_i, min_j,max_i,max_j, i, j);
          }
          else
          {
            result = computeGlobalVisibilityIndex(master_array, min_i, min_j,max_i,max_j, i, j);
          }
          
          //ROS_INFO("checkpoint %d %d", visible_cells, invisible_cells);
          costmap_[index] = static_cast<int> (result);


          
          // ROS_INFO("cells %d %d %d", visible_cells, invisible_cells, costmap_[index]);

          
  
      
        }
      }
      
      // ROS_INFO("end vision");
      // if(active_layer_)
      updateWithAddition(master_grid, min_i, min_j, max_i, max_j);
      // for (int t = min_j; t < max_j; t++) 
      // {
      //   for (int f = min_i; f < max_i; f++) 
      //   {
      //     int index2=getIndex(f, t);
      //     std::cout<<(int)master_array[index2]<<" ";
      //   }
      //   std::cout<<std::endl;
      // }
    
      // ROS_INFO("=========");

      // for (int t = min_j; t < max_j; t++) 
      // {
      //   for (int f = min_i; f < max_i; f++) 
      //   {
      //     int index2=getIndex(f, t);
      //     std::cout<<(int)costmap_[index2]<<" ";
      //   }
      //   std::cout<<std::endl;
      // }
    } 

    float VisionLayer::computeLocalVisibilityIndex(unsigned char * master_array, int min_i, int min_j,int max_i,int max_j, int i, int j)
    {
      
      int k, l, index, index2;
      index = getIndex(i, j);
      unsigned char cost;

      unsigned int visibility_window_size = 30;

      int cell_raytrace_range = 20;

      cost = 200;

      unsigned int visible_cells = 0;
      unsigned int invisible_cells = 0;

      //unsigned char * master_array = master_grid.getCharMap();
      unsigned char * local_map = new unsigned char[3600]; 

      for (int a = min_j; a < max_j; a++) 
      {
        for (int b = min_i; b < max_i; b++) 
        {
          index2=getIndex(b, a);
          local_map[index2] = master_array[index2];
        }
      }

      // ROS_INFO("checkpoint");
      // unsigned char *map_copy = new unsigned char(*master_array);   
      

      VisionCell vc(local_map, cost, visible_cells, invisible_cells);
      for (k = min_j; k < max_j; k++) 
      {
        l = min_i;
                  
        // and finally... we can execute our trace to clear obstacles along that line
        raytraceLine(vc, i, j, l, k, cell_raytrace_range);

        l = max_i-1;
                  
        // and finally... we can execute our trace to clear obstacles along that line
        raytraceLine(vc, i, j, l, k, cell_raytrace_range);
      }

      for (int l = min_i; l < max_i; l++) 
      {
        k = min_j;
                  
        // and finally... we can execute our trace to clear obstacles along that line
        raytraceLine(vc, i, j, l, k, cell_raytrace_range);

        k = max_j-1;

        // and finally... we can execute our trace to clear obstacles along that lin
        raytraceLine(vc, i, j, l, k, cell_raytrace_range);
      }
      float result = static_cast<float>(visible_cells) / (visible_cells + invisible_cells);

      result = (1.0 - result) * 600.0; //*252.0

      if(result > 252)
        result = 252;
      delete local_map;
      return result;
    }


    float VisionLayer::computeGlobalVisibilityIndex(unsigned char * master_array, int min_i, int min_j,int max_i,int max_j, int i, int j)
    {
      int k, l, index, index2, index3;
      index = getIndex(i, j);
      unsigned char cost;

      unsigned int visibility_window_size = 30;

      int cell_raytrace_range = 15;

      cost = 200;

      unsigned int visible_cells = 0;
      unsigned int invisible_cells = 0;

      //unsigned char * master_array = master_grid.getCharMap();
      unsigned char * local_map = new unsigned char[3600]; 

      int window_min_i, window_max_i, window_min_j, window_max_j, local_i, local_j;
      
      window_min_i = ((i - 15) > min_i) ? (i-15): min_i;
      window_max_i = ((i + 15) < max_i) ? (i+15): max_i;
      window_min_j = ((j - 15) > min_j) ? (j-15): min_j;
      window_max_j = ((j + 15) < max_j) ? (j+15): max_j;
      
      //ROS_INFO("checkpoint");
      for (int a = window_min_j, c=0; a < window_max_j; a++, c++) 
      {
        for (int b = window_min_i, d=0; b < window_max_i; b++, d++) 
        {
          index2 = getIndex(b, a);
          index3 = c * (window_max_j - window_min_j) + d; //calculo do getindex
          if(a == j && b == i)
          {
            //ROS_INFO("%d %d %d %d %d %d", a,b,c,d,i,j);
            local_j = c;
            local_i = d;
          }
          //ROS_INFO("%d %d", index2, index3);
          local_map[index3] = master_array[index2];
        }
      }

      //ROS_INFO("checkpoint");
      // unsigned char *map_copy = new unsigned char(*master_array);

      // ROS_INFO("LOCAL MAP");

     //for (int t = 0; t <  window_max_i - window_min_i +1; t++) 
     //{
       //for (int f = 0; f < window_max_j - window_min_j +1; f++) 
       //{
         //int index2=t*60 +f;
         //std::cout<<(int)local_map[index2]<<" ";
       //}
       //std::cout<<std::endl;
     //}

      VisionCell vc(local_map, cost, visible_cells, invisible_cells);
      for (k = 0; k < (window_max_j - window_min_j); k++) 
      { //TODO get i j points in the local costmap
          l = 0;
          //ROS_INFO("%d %d %d %d %d %d", local_i,local_j,window_max_j - window_min_j, window_max_i - window_min_i, l,k);
          // and finally... we can execute our trace to clear obstacles along that line
          raytraceLine(vc, local_i, local_j, l, k, cell_raytrace_range);

          l = window_max_i - window_min_i -1;

          //ROS_INFO("%d %d %d %d %d %d", local_i,local_j,window_max_j - window_min_j, window_max_i - window_min_i, l,k);
          
          // and finally... we can execute our trace to clear obstacles along that line
          raytraceLine(vc, local_i, local_j, l, k, cell_raytrace_range);

        
      }

      //ROS_INFO("checkpoint");
      for (l = 0; l < (window_max_i - window_min_i); l++) 
      {
        k = 0;
        //ROS_INFO("%d %d %d %d %d %d", local_i,local_j,window_max_j - window_min_j, window_max_i - window_min_i, l,k);
        // and finally... we can execute our trace to clear obstacles along that line
        raytraceLine(vc, local_i, local_j, l, k, cell_raytrace_range);

        k = window_max_j - window_min_j -1;

        //ROS_INFO("%d %d %d %d %d %d", local_i,local_j,window_max_j - window_min_j, window_max_i - window_min_i, l,k);
        // and finally... we can execute our trace to clear obstacles along that line
        raytraceLine(vc, local_i, local_j, l, k, cell_raytrace_range);
      }
      //ROS_INFO("checkpoint");
      float result = static_cast<float>(visible_cells) / (visible_cells + invisible_cells);

      result = (1.0 - result) * 252.0;

      if(result > 252)
        result = 252;
      delete local_map;
      return result;
    }

}  // namespace vision_layer

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.



