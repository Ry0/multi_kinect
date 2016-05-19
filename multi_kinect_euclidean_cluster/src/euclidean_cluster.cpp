#include <euclidean_cluster.hpp>
using namespace pcl;

EuclideanCluster::EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh), rate_(n.param("loop_rate", 50)),
    frame_id_(n.param<std::string>("resized_pc_frame_id", "/resized_pc_frame"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/merged_cloud"), 1, &EuclideanCluster::EuclideanCallback, this);
  euclidean_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("resized_pc_topic_name", "/trans_pointcloud"), 1);
}

void EuclideanCluster::EuclideanCallback(const sensor_msgs::PointCloud2::ConstPtr &source_pc) {

  //点群をKinect座標系からWorld座標系に変換
  //変換されたデータはtrans_pcに格納される．
  sensor_msgs::PointCloud2 trans_pc;
  try{
    pcl_ros::transformPointCloud("world",
                                 *source_pc,
                                 trans_pc,
                                 tflistener);
    }
    catch (tf::ExtrapolationException e){
         ROS_ERROR("pcl_ros::transformPointCloud %s",e.what());
  }
  // trans_pc.header.stamp = ros::Time::now();
  // trans_pc.header.frame_id = "world";
  // euclidean_cluster_pub_.publish(trans_pc);

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<PointXYZ> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);
  pcl::PointCloud<PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<PointXYZ>(pcl_source));

  // 点群の中からnanを消す
  // std::vector<int> dummy;
  // pcl::removeNaNFromPointCloud(*pcl_source_ptr, *pcl_source_ptr, dummy);

  // 平面をしきい値で除去する→Cropboxで
  pcl::PointXYZ min, max;
  min.x = -1; max.x = 1;
  min.y = -1; max.y = 1;
  min.z = 0.01; max.z = 1;
  CropBox(pcl_source_ptr, min, max);

  sensor_msgs::PointCloud2 cloud_filtered_pc2;
  pcl::toROSMsg(*pcl_source_ptr, cloud_filtered_pc2);
  trans_pc.header.stamp = ros::Time::now();
  trans_pc.header.frame_id = "world";
  euclidean_cluster_pub_.publish(cloud_filtered_pc2);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pcl_source_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pcl_source_ptr);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (pcl_source_ptr->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    j++;
  }

  // // int clusterLength = clusterIndices.size();

  ROS_INFO("Found %lu clusters:", cluster_indices.size());

  // // Empty Buffer
  cluster_indices.clear();
}

bool EuclideanCluster::CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               pcl::PointXYZ min, pcl::PointXYZ max) {
  bool hr = true;
  Eigen::Vector4f minPoint;

  minPoint[0] = min.x; // define minimum point x
  minPoint[1] = min.y; // define minimum point y
  minPoint[2] = min.z; // define minimum point z

  Eigen::Vector4f maxPoint;
  maxPoint[0] = max.x; // define max point x
  maxPoint[1] = max.y; // define max point y
  maxPoint[2] = max.z; // define max point z

  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = 0;
  boxTranslatation[1] = 0;
  boxTranslatation[2] = 0;

  Eigen::Vector3f boxRotation;
  boxRotation[0] = 0; // rotation around x-axis
  boxRotation[1] = 0; // rotation around y-axis
  boxRotation[2] = 0; // in radians rotation around z-axis. this rotates your
                      // cube 45deg around z-axis.

  Eigen::Affine3f boxTransform;

  pcl::CropBox<pcl::PointXYZ> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTranslation(boxTranslatation);
  cropFilter.setRotation(boxRotation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>());

  cropFilter.filter(*cloud_filtered);

  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*cloud_filtered, *cloud);

  return hr;
}


void EuclideanCluster::run()
{
  while(nh_.ok()){
    ros::spinOnce();
    rate_.sleep();
  }
}
