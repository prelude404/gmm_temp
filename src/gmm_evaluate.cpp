#include "ros/ros.h"
#include "gmm_test/gmm_test.h"

//ros初始化部分
ros::Subscriber cloud_sub_original;
ros::Subscriber cloud_sub_reconstruct;
int AGENT_ID = 0;
double grid_size = 0.04;

//算法初始化部分
int mark_if_gotten_OriginalCloud = 0;//判断是否收到原始点云
std::vector<Eigen::Vector3d> Voxel_cloud_original;

//回调函数申明
void OriginalCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
void ReconstructCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //执行节点初始化
    ros::init(argc,argv,"GMM_test");
	ros::NodeHandle nh("~");

	nh.getParam("grid_size",grid_size);

	cloud_sub_original=nh.subscribe("/gmm_cloud", 1, OriginalCallback);
	cloud_sub_reconstruct=nh.subscribe("/gmm_cloud_sample", 1, ReconstructCallback);
	
    ros::Duration(0.1).sleep();
	ros::spin();
    return 0;
}


void OriginalCallback(const sensor_msgs::PointCloud2ConstPtr &cloud){
	if(mark_if_gotten_OriginalCloud == 0) {}
	else{return;}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud, *cloud_in);

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	static float res = grid_size;
	sor.setLeafSize(res, res, res);
	sor.filter(*cloud_in);	

	std::vector<Eigen::Vector3d> Voxel_cloud_tmp;
	for(int i = 0;i<cloud_in->points.size();i++){
		Eigen::Vector3d voxel_tmp;
		voxel_tmp(0) = ceil(cloud_in->points[i].x/grid_size);
		voxel_tmp(1) = ceil(cloud_in->points[i].y/grid_size);
		voxel_tmp(2) = ceil(cloud_in->points[i].z/grid_size);
		Voxel_cloud_tmp.push_back(voxel_tmp);
	}
	Voxel_cloud_original = Voxel_cloud_tmp;

	mark_if_gotten_OriginalCloud = 1;
}

void ReconstructCallback(const sensor_msgs::PointCloud2ConstPtr &cloud){
	if(mark_if_gotten_OriginalCloud == 1) {}
	else{return;}

	//把获取的GMM重构点云也体素滤波，方便生成栅格
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud, *cloud_in);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	static float res = grid_size;
	sor.setLeafSize(res, res, res);
	sor.filter(*cloud_in);	

	//::cout<<"size_gmm"<<cloud_in->points.size()<<std::endl;
	//std::cout<<"size_origin"<<Voxel_cloud_original.size()<<std::endl;

	std::vector<Eigen::Vector3d> Voxel_cloud_gmm;
	for(int i = 0;i<cloud_in->points.size();i++){
		Eigen::Vector3d voxel_tmp;
		voxel_tmp(0) = ceil(cloud_in->points[i].x/grid_size);
		voxel_tmp(1) = ceil(cloud_in->points[i].y/grid_size);
		voxel_tmp(2) = ceil(cloud_in->points[i].z/grid_size);
		Voxel_cloud_gmm.push_back(voxel_tmp);
	}

	//比较两个点云的差距
	int only_0 = 0;
	int only_1 = 0;
	int both_01 = 0;
	for(int i = 0;i<Voxel_cloud_original.size();i++){
		for(int j = 0;j<Voxel_cloud_gmm.size();j++){
			if(pow(Voxel_cloud_gmm[j](0) - Voxel_cloud_original[i](0),2) +
				pow(Voxel_cloud_gmm[j](1) - Voxel_cloud_original[i](1),2) +
				pow(Voxel_cloud_gmm[j](2) - Voxel_cloud_original[i](2),2) < 1){
				both_01++;
				break;
			}
		}
	}
	only_0 = Voxel_cloud_original.size() - both_01;
	only_1 = Voxel_cloud_gmm.size() - both_01;

	std::cout<<"rate = "<<2*(double)both_01/(2*(double)both_01+(double)only_0+(double)only_1)<<"    ";
	std::cout<<only_0<<","<<only_1<<","<<both_01<<","<<std::endl;
	
	mark_if_gotten_OriginalCloud = 0;
}

std::vector<Eigen::Vector3d> Cal_voxel_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud){




}
