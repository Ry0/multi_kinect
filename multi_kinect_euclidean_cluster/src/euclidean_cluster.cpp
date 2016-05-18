#include <euclidean_cluster.hpp>
using namespace std;
using namespace ros;
using namespace pcl;

EuclideanCluster::EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh), rate_(n.param("loop_rate", 10)),
    frame_id_(n.param<std::string>("resized_pc_frame_id", "/resized_pc_frame"))
{
  // leaf_size_x_ = n.param("leaf_size_x", 0.3);
  // leaf_size_y_ = n.param("leaf_size_y", 0.3);
  // leaf_size_z_ = n.param("leaf_size_z", 0.3);

  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/merged_cloud"), 1, &EuclideanCluster::EuclideanCallback, this);
  // resized_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("resized_pc_topic_name", "/resized_pointcloud"), 1);
}

void EuclideanCluster::EuclideanCallback(const sensor_msgs::PointCloud2::ConstPtr &source_pc) {

  //点群をrobot座標系からglobal座標系に変換
  //変換されたデータはlaserPointsGlobalに格納される．
  sensor_msgs::PointCloud2 trans_pc;
  tflistener.transformPointCloud("kinect_first/kinect2_rgb_optical_frame",*source_pc->header.stamp,*source_pc,"world",trans_pc);

  pcl::PointCloud<PointXYZ> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);
  pcl::PointCloud<PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<PointXYZ>(pcl_source));


    std::vector<int> dummy;
    pcl::removeNaNFromPointCloud(*pcl_source_ptr, *pcl_source_ptr, dummy);





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

    // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    // std::stringstream ss;
    // ss << "cloud_cluster_" << j << ".pcd";
    // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  // // int clusterLength = clusterIndices.size();

  ROS_INFO("Found %lu clusters:", cluster_indices.size());

  // // Empty Buffer
  cluster_indices.clear();
}

void EuclideanCluster::run()
{
  while(nh_.ok()){
    ros::spinOnce();
    rate_.sleep();
  }
}
