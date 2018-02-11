#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>


class ClusterRGB {

public:

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterRGB {new pcl::PointCloud<pcl::PointXYZRGB>};
	unsigned short index;
	Eigen::Vector4f centroid;

};


class Callback {

public:
	
	Callback(ros::NodeHandle nh) {
		
		sub = nh.subscribe ("/Sensor/points", 1, &Callback::pclCallback, this);
		pub_cluster = nh.advertise<sensor_msgs::PointCloud2> ("/clustered_pcl", 1000);
		pub_marker = nh.advertise<visualization_msgs::MarkerArray> ("visualization_marker_array", 1000);
	}

	float distance(ClusterRGB obj1, ClusterRGB obj2) {

		return std::sqrt(std::pow((obj1.centroid[0] - obj2.centroid[0]), 2) + 
						 std::pow((obj1.centroid[1] - obj2.centroid[1]), 2) +
						 std::pow((obj1.centroid[2] - obj2.centroid[2]), 2));
	}
	
	void pclCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
		
		std::string frame_id = cloud_msg->header.frame_id;
  		pcl::PCLPointCloud2 pcl2;
		pcl_conversions::toPCL(*cloud_msg, pcl2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(pcl2, *cloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

		pcl::PassThrough<pcl::PointXYZ> passZ;
		passZ.setInputCloud (cloud);
		passZ.setFilterFieldName ("z");
		passZ.setFilterLimits (-1.5, 2.5);
		passZ.filter (*cloud_filtered);

		pcl::PassThrough<pcl::PointXYZ> passY;
		passY.setInputCloud (cloud_filtered);
		passY.setFilterFieldName ("y");
		passY.setFilterLimits (-8.5, 4.0);
		passY.filter (*cloud_filtered);
	
		double avg_dist = 0.0;
		unsigned int size = 0;
		float tolerance = 0.0;

		for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_filtered->begin();it!=cloud_filtered->end();it++) {
			avg_dist += std::sqrt(std::pow(it->x, 2) + std::pow(it->y, 2) + std::pow(it->z, 2));
			size+=1;
		}

		avg_dist /=  size;

		if(avg_dist<=7) {
			tolerance = 0.65;
		} else if(avg_dist<=10) {
			tolerance = 0.8;
		} else if(avg_dist<=12) {
			tolerance = 0.95;
		} else if(avg_dist<=15) {
			tolerance = 1.1;
		} else {
			tolerance = 1.25;
		}

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (tolerance);
		ec.setMinClusterSize (20);
		ec.setMaxClusterSize (10000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_filtered);
		ec.extract (cluster_indices);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusters (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
		ClusterRGB* obj = new ClusterRGB();

		std::vector<ClusterRGB> cluster_curr;
		std::vector<ClusterRGB> cluster_prev;
	
		uint32_t r,g,b;
		unsigned short j(0);

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
				cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
			}
			
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			pcl::copyPointCloud(*cloud_cluster, *(obj->clusterRGB));

			int32_t rgb = (static_cast<uint32_t>(r) << (rand() % 256) | static_cast<uint32_t>(g) << (rand() % 256) | static_cast<uint32_t>(b << (rand() % 256)));

			for (auto &p: obj->clusterRGB->points) {
				p.rgb = rgb;
			}
			
			Eigen::Vector4f centroid_rgb; 
			*clusters += *(obj->clusterRGB);
			pcl::compute3DCentroid(*(obj->clusterRGB), centroid_rgb);
			obj->index = j;
			obj->centroid = centroid_rgb;
			cluster_curr.push_back(*obj);
			j++;
		}

		unsigned short size_curr(cluster_curr.size());

		visualization_msgs::MarkerArray marker;
		
		if (cluster_prev.size()==0) {

			cluster_prev = cluster_curr;
			marker.markers.resize(size_curr);

			for (int i=0;i<size_curr;i++) {
				marker.markers[i].header.frame_id = frame_id;
				marker.markers[i].header.stamp = ros::Time();
				marker.markers[i].ns = "cluster_track";
				marker.markers[i].id = cluster_curr[i].index;
				marker.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
				marker.markers[i].text = std::to_string(cluster_curr[i].index);
				marker.markers[i].action = visualization_msgs::Marker::ADD;
				marker.markers[i].pose.position.x = cluster_curr[i].centroid[0];
				marker.markers[i].pose.position.y = cluster_curr[i].centroid[1];
				marker.markers[i].pose.position.z = cluster_curr[i].centroid[2];
				marker.markers[i].pose.orientation.x = 0.0;
				marker.markers[i].pose.orientation.y = 0.0;
				marker.markers[i].pose.orientation.z = 0.0;
				marker.markers[i].pose.orientation.w = 1.0;
				marker.markers[i].lifetime = ros::Duration(0.3);
				// marker.markers[i].scale.x = 1.0;
				// marker.markers[i].scale.y = 1.0;
				marker.markers[i].scale.z = 1.5;
				marker.markers[i].color.a = 1.0; 
				// marker.markers[i].color.r = 204.0;
				// marker.markers[i].color.g = 255.0;
				// marker.markers[i].color.b = 0.0;
				marker.markers[i].color.r = 0.8f;
				marker.markers[i].color.g = 1.0f;
				marker.markers[i].color.b = 0.0f;
			}

		} else {

			unsigned short size_prev(cluster_prev.size());

			if (size_curr!=0) {

				if (size_curr<=size_prev) {

					std::vector<int> not_found;

					for (int i=0;i<size_curr;i++) {
						float min_dist = std::numeric_limits<float>::max();
						unsigned short index(0);
						unsigned short pos;
						for(int j=0;j<cluster_prev.size();j++){
							float dist = distance(cluster_curr[i], cluster_prev[j]);
							if(dist<=min_dist && abs(min_dist - dist)<=5.0){ 
								min_dist = dist;
								index = cluster_prev[j].index;
								pos = j;
							}
						}
						cluster_prev.erase(cluster_prev.begin() + pos);
						cluster_curr[i].index = index;
					}

				} else {

					int i;

					for (i=0;i<size_curr;i++) {
						float min_dist = std::numeric_limits<float>::max();
						unsigned short index(0);
						unsigned short pos;
						for(int j=0;j<cluster_prev.size();j++){
							float dist = distance(cluster_curr[i], cluster_prev[j]);
							if(dist<=min_dist && abs(min_dist - dist)<=5.0){
								min_dist = dist;
								index = cluster_prev[j].index;
								pos = j;
							}
						}
						cluster_prev.erase(cluster_prev.begin() + pos);
						cluster_curr[i].index = index;
					}

					for (int k=i;k<size_curr;k++) {
						cluster_curr[i].index = rand() + 1000;
					}
				}

				marker.markers.resize(size_curr);

				for (int i=0;i<size_curr;i++) {
					marker.markers[i].header.frame_id = frame_id;
					marker.markers[i].header.stamp = ros::Time();
					marker.markers[i].ns = "cluster_track";
					marker.markers[i].id = cluster_curr[i].index;
					marker.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					marker.markers[i].text = std::to_string(cluster_curr[i].index);
					marker.markers[i].action = visualization_msgs::Marker::ADD;
					marker.markers[i].pose.position.x = cluster_curr[i].centroid[0];
					marker.markers[i].pose.position.y = cluster_curr[i].centroid[1];
					marker.markers[i].pose.position.z = cluster_curr[i].centroid[2];
					marker.markers[i].pose.orientation.x = 0.0;
					marker.markers[i].pose.orientation.y = 0.0;
					marker.markers[i].pose.orientation.z = 0.0;
					marker.markers[i].pose.orientation.w = 1.0;
					marker.markers[i].lifetime = ros::Duration(0.3);
					// marker.markers[i].scale.x = 1.0;
					// marker.markers[i].scale.y = 1.0;
					marker.markers[i].scale.z = 1.5;
					marker.markers[i].color.a = 1.0; 
					// marker.markers[i].color.r = 204.0;
					// marker.markers[i].color.g = 255.0;
					// marker.markers[i].color.b = 0.0;
					marker.markers[i].color.r = 0.8f;
					marker.markers[i].color.g = 1.0f;
					marker.markers[i].color.b = 0.0f;
				}

			} else {

				ROS_INFO("NO CLUSTERS FOUND");
			}

			cluster_prev = cluster_curr;
		}
		
		pub_marker.publish(marker);

		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*clusters, output);
		output.header.frame_id = frame_id;
		output.header.stamp = ros::Time::now();

		pub_cluster.publish(output);
	}
  
private:
	
	ros::Subscriber sub;
	ros::Publisher pub_cluster;
	ros::Publisher pub_marker;

};


int main(int argc, char **argv) {
	
	srand((unsigned)time(0));
	ros::init(argc, argv, "pointCloudCluster");
	ros::NodeHandle nh;

	Callback obj(nh);

	ros::spin();
	return 0;
}
