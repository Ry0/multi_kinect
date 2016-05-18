#include <euclidean_cluster.hpp>
using namespace pcl;

EuclideanCluster::EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh), rate_(n.param("loop_rate", 10)),
    frame_id_(n.param<std::string>("resized_pc_frame_id", "/resized_pc_frame"))
{
  // leaf_size_x_ = n.param("leaf_size_x", 0.3);
  // leaf_size_y_ = n.param("leaf_size_y", 0.3);
  // leaf_size_z_ = n.param("leaf_size_z", 0.3);

  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/merged_cloud"), 1, &EuclideanCluster::EuclideanCallback, this);
  euclidean_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("resized_pc_topic_name", "/trans_pointcloud"), 1);
}

void EuclideanCluster::EuclideanCallback(const sensor_msgs::PointCloud2::ConstPtr &source_pc) {

  //点群をKinect座標系からWorld座標系に変換
  //変換されたデータはtrans_pcに格納される．
  sensor_msgs::PointCloud2 trans_pc;
  pcl_ros::transformPointCloud("world",
                               *source_pc,
                               trans_pc,
                               tflistener);
  trans_pc.header.stamp = ros::Time::now();
  trans_pc.header.frame_id = "world";
  euclidean_cluster_pub_.publish(trans_pc);

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<PointXYZ> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);
  pcl::PointCloud<PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<PointXYZ>(pcl_source));

  // 点群の中からnanを消す
  std::vector<int> dummy;
  pcl::removeNaNFromPointCloud(*pcl_source_ptr, *pcl_source_ptr, dummy);

  // 平面をしきい値で除去する→
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pcl_source_ptr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.1, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  // pass.setInputCloud (cloud_filtered);
  // pass.setFilterFieldName ("x");
  // pass.setFilterLimits (-1, 1);
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*cloud_filtered);

  // pass.setInputCloud (cloud_filtered);
  // pass.setFilterFieldName ("y");
  // pass.setFilterLimits (-1, 1);
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*cloud_filtered);


  sensor_msgs::PointCloud2 cloud_filtered_pc2;
  pcl::toROSMsg(*cloud_filtered, cloud_filtered_pc2);
  trans_pc.header.stamp = ros::Time::now();
  trans_pc.header.frame_id = "world";
  euclidean_cluster_pub_.publish(cloud_filtered_pc2);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
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
