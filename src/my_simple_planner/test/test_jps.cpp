#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include "local_perception/sdf_map.h"
#include "jps_planner/jps_planner.h"

using namespace std;

shared_ptr<SDFMap> sdf_map;
std::unique_ptr<JPSPlanner3D> planner_ptr;
nav_msgs::Odometry odom;
ros::Publisher vis_traj_pub;
ros::Publisher vis_debug;


void recvGoalCallback(const geometry_msgs::PoseStamped &wp);
void recvOdomCallback(const nav_msgs::OdometryConstPtr &odom_);
void visualizeESDFGrid(const ros::TimerEvent &event);
void convertSDFtoMapUtil(std::shared_ptr<JPS::MapUtil<3>>& map_util);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_esdf");
    ros::NodeHandle nh("~");

    ros::Subscriber pts_sub = nh.subscribe("/test_planner/goal", 1, recvGoalCallback);
    ros::Subscriber odom_sub = nh.subscribe("/test_planner/odom", 1, recvOdomCallback);
    vis_traj_pub = nh.advertise<visualization_msgs::Marker>("RRTStar_path_vis", 1);
    vis_debug = nh.advertise<sensor_msgs::PointCloud2>("debug_vis", 1);

    sdf_map.reset(new SDFMap);
    sdf_map->initMap(nh);

    planner_ptr.reset(new JPSPlanner3D(true));
    planner_ptr->setParam(nh);

    while(!sdf_map->hasDepthObservation()){
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // convert sdfmap into maputil
    std::shared_ptr<JPS::MapUtil<3>> map_util = std::make_shared<JPS::MapUtil<3>>();
    convertSDFtoMapUtil(map_util);
    planner_ptr->setMapUtil(map_util); // Set collision checking function
    planner_ptr->updateMap();

    ros::Timer vis_esdf_timer = nh.createTimer(ros::Duration(1.0), visualizeESDFGrid);

    ros::spin();
    return 0;
}

void convertSDFtoMapUtil(std::shared_ptr<JPS::MapUtil<3>>& map_util)
{
    // get the map size
    decimal_t resolution = sdf_map->getResolution();
    Eigen::Vector3d origin = sdf_map->getOrigin();
    Vecf<3> origin_ = {origin.x(), origin.y(), origin.z()};
    Eigen::Vector3d size = sdf_map->getSize();
    Veci<3> size_ = {static_cast<int>(size.x()/resolution), 
                     static_cast<int>(size.y()/resolution), 
                     static_cast<int>(size.z()/resolution)};
    
    std::vector<signed char> temp_map;
    temp_map.resize(size_.x() * size_.y() * size_.z());
    std::cout << "size: " << size_.transpose() << std::endl;
    // get the map

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointXYZ pt;

    for (int z = 0; z < size_.z(); z++)
    {
        for (int y = 0; y < size_.y(); y++)
        {
            for (int x = 0; x < size_.x(); x++)
            {
                Eigen::Vector3d pos = origin + Eigen::Vector3d(x * resolution, y * resolution, z * resolution);
                // if(pos.x() < min_pos_x) min_pos_x = pos.x();
                // if(pos.y() < min_pos_y) min_pos_y = pos.y();
                // if(pos.z() < min_pos_z) min_pos_z = pos.z();
                // if(pos.x() > max_pos_x) max_pos_x = pos.x();
                // if(pos.y() > max_pos_y) max_pos_y = pos.y();
                // if(pos.z() > max_pos_z) max_pos_z = pos.z();
                // temp_map[x + y * size_.x() + z * size_.x() * size_.y()] = sdf_map->getInflateOccupancy(pos);
                Eigen::Vector3i id = {x, y, z};
                int occ_id = sdf_map->getInflateOccupancy(id);
                int occ_pos = sdf_map->getInflateOccupancy(pos);
                // if(occ_id != occ_pos){
                //     cnt++;
                //     std::cout << "id: " << occ_id << " pos: " << occ_pos << " cnt " << cnt << std::endl;
                // }
                if(occ_id > 0 || pos.z() <= 0.15 || pos.z() >= 0.25) occ_id=100;
                temp_map[x + y * size_.x() + z * size_.x() * size_.y()] = occ_id;
                if (occ_id > 0){
                    pt.x = pos.x();
                    pt.y = pos.y();
                    pt.z = pos.z();
                    if(pos.z() <= 0.15 || pos.z() >= 0.25)
                        continue;
                    cloud.points.push_back(pt);
                }
            }
        }
    }
    std::cout << "points size " << cloud.points.size() << std::endl;
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    vis_debug.publish(cloud_msg);

    // std::cout << "min_pos_x: " << min_pos_x << " min_pos_y: " << min_pos_y << " min_pos_z: " << min_pos_z << std::endl;
    // std::cout << "max_pos_x: " << max_pos_x << " max_pos_y: " << max_pos_y << " max_pos_z: " << max_pos_z << std::endl;

    map_util->setMap(origin_, size_, temp_map, resolution);
}


void visualizeESDFGrid(const ros::TimerEvent &event)
{
    if (!sdf_map->hasDepthObservation())
        return;

    sdf_map->publishMap();
    sdf_map->publishMapInflate();
    sdf_map->publishUpdateRange();
    sdf_map->publishESDF();
}


void recvGoalCallback(const geometry_msgs::PoseStamped &wp)
{
    auto pos = odom.pose.pose.position;
    Vecf<3> start = {pos.x, pos.y, 0.2};
    pos = wp.pose.position;
    Vecf<3> goal = {pos.x, pos.y, 0.2};

    ROS_WARN("recv goal start planning!");
    bool valid_jps = planner_ptr->plan(start, goal, 1, true);
    ROS_WARN("finish planning!");
    
    if(valid_jps){
        auto path_jps = planner_ptr->getPath();
        // for(const auto& it: path_jps)
        //     std::cout << it.transpose() << std::endl;
    
        planner_ptr->publishAll();
    }
}

void recvOdomCallback(const nav_msgs::OdometryConstPtr &odom_)
{
  odom = *odom_;
}