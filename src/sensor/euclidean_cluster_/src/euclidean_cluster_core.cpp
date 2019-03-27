#include "euclidean_cluster_core.h"

EuClusterCore::EuClusterCore(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{

    seg_distance_ = {15, 25, 35};
    cluster_distance_ = {0.4, 0.8, 1.4, 1.8};
    sub_point_cloud_ = nh.subscribe("/pandar_points", 5, &EuClusterCore::point_cb, this);

	pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 5);
    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);
    
    private_nh.param<float>("x_max",x_max_,1.0);
    private_nh.param<float>("x_min",x_min_,1.0);
    private_nh.param<float>("y_max",y_max_,1.0);
    private_nh.param<float>("y_min",y_min_,1.0);

    ros::spin();
}

EuClusterCore::~EuClusterCore() {}

//去除 地面以及高于 2m 物体
void EuClusterCore::clip_filter(double clip_height, double clip_low,  const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZ> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
    	//double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);
    	//去除过高、过低、过远点云
        if (  (in->points[i].z > clip_height) || (in->points[i].z < clip_low) || (in->points[i].x >x_max_) || (in->points[i].x<x_min_) || (in->points[i].y>y_max_) || (in->points[i].y<y_min_))
        {
            indices.indices.push_back(i);
        }
        else if ( (in->points[i].x<1.6) &&  (in->points[i].x>-1.5 ) && (abs(in->points[i].y)<0.6) )  //去除过近的点云 //
        {
        
        	indices.indices.push_back(i);
        }
        
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}


/*void EuClusterCore::publish_cloud(const ros::Publisher &in_publisher,const,pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}
*/

void EuClusterCore::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
}

void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                    double in_max_cluster_distance, std::vector<Detected_Obj> &obj_list)
{

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //create 2d pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    //make it flat
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud_2d);
    euclid.setClusterTolerance(in_max_cluster_distance);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);

    for (size_t i = 0; i < local_indices.size(); i++)
    {
        // the structure to save one detected object
        Detected_Obj obj_info;

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y;
            p.z = in_pc->points[*pit].z;

            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;

            if (p.x < min_x)
                min_x = p.x;
            if (p.y < min_y)
                min_y = p.y;
            if (p.z < min_z)
                min_z = p.z;
            if (p.x > max_x)
                max_x = p.x;
            if (p.y > max_y)
                max_y = p.y;
            if (p.z > max_z)
                max_z = p.z;
        }

        //min, max points
        obj_info.min_point_.x = min_x;
        obj_info.min_point_.y = min_y;
        obj_info.min_point_.z = min_z;

        obj_info.max_point_.x = max_x;
        obj_info.max_point_.y = max_y;
        obj_info.max_point_.z = max_z;

        //calculate centroid, average
        if (local_indices[i].indices.size() > 0)
        {
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();
        }

        //calculate bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;
		
		double volume = length_*width_*height_; //体积
		

        obj_info.bounding_box_.header = point_cloud_header_;

        obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2;
        obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
        obj_info.bounding_box_.pose.position.z = obj_info.min_point_.z + height_ / 2;
        
        double x_value=abs(obj_info.min_point_.x)-2.0;
        double y_value=abs(obj_info.bounding_box_.pose.position.y);

		double distance = sqrt( x_value*x_value+ y_value*y_value );
	

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

		if(  volume<2 && volume >0.4 )
		{
			obj_info.bounding_box_.label=1;   //人
			printf("label:%d  diatance:%f \r\n",obj_info.bounding_box_.label,distance);
		}
		else if(volume >2 && volume <10 )
		{
			obj_info.bounding_box_.label=2;  //车
			printf("label:%d  diatance:%f \r\n",obj_info.bounding_box_.label,distance);
			
		}
		else
		{		
			obj_info.bounding_box_.label=0; //未知
		}

        obj_list.push_back(obj_info);
    }
}

void EuClusterCore::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list)
{
    //cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    //in this way, the points farther in the pc will also be clustered

    //0 => 0-15m d=0.5
    //1 => 15-30 d=1
    //2 => 30-45 d=1.6
    //3 => 45---- d=2.1
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(4);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        if (origin_distance < seg_distance_[0])
        {
            segment_pc_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1])
        {
            segment_pc_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2])
        {
            segment_pc_array[2]->points.push_back(current_point);
        }
        else 
        {
            segment_pc_array[3]->points.push_back(current_point);
        }
        
    }

    std::vector<pcl::PointIndices> final_indices;
    std::vector<pcl::PointIndices> tmp_indices;

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        cluster_segment(segment_pc_array[i], cluster_distance_[i], obj_list);
    }
}

void EuClusterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
     
    pcl::PointCloud<pcl::PointXYZ>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);//按xyz距离远近 过滤点云
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>); //降采样


    point_cloud_header_ = in_cloud_ptr->header;

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
   
    
	clip_filter(CLIP_HEIGHT, CLIP_LOW, current_pc_ptr, cliped_pc_ptr);
	
	sensor_msgs::PointCloud2 pub_pc;
	pcl::toROSMsg(*cliped_pc_ptr, pub_pc);
    pub_pc.header = in_cloud_ptr->header;
    pub_filtered_points_.publish(pub_pc);


	 // down sampling the point cloud before cluster
    voxel_grid_filer(cliped_pc_ptr, filtered_pc_ptr, LEAF_SIZE);

    std::vector<Detected_Obj> global_obj_list;
    cluster_by_distance(filtered_pc_ptr, global_obj_list);

    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for (size_t i = 0; i < global_obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    }
    bbox_array.header = point_cloud_header_;

    pub_bounding_boxs_.publish(bbox_array);
}
