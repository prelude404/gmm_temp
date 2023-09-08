#include "ros/ros.h"
//#include "gmm_test/gmm_test.h"
#include "gmm_test/gmm_tree_fitting.h"
#include "gmm_test/gmm_sample.h"
#include "gmm_test/GMM.h"

void gmmCallback(gmm_test::GMM msg);
void headangleCallback(geometry_msgs::PoseStamped msg);
void placeCallback(geometry_msgs::PoseStamped msg);
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //执行节点初始化
    ros::init(argc,argv,"GMM_test");
	ros::NodeHandle nh("~");

	nh.getParam("agent_id",AGENT_ID);
	nh.getParam("x",initial_offset_x);
	nh.getParam("y",initial_offset_y);
	nh.getParam("z",initial_offset_z);
	nh.getParam("near_distance",near_distance);
	nh.getParam("far_distance",far_distance);
	nh.getParam("grid_size",grid_size);
	nh.getParam("Z_boundary",Z_boundary);
	nh.getParam("R_filter",R_filter);
	nh.getParam("N_filter",N_filter);
	nh.getParam("max_number",max_number);
	nh.getParam("EM_begin_number",EM_begin_number);

	current_p<<0,0,0;
	current_q<<0,0,1,-1;

	GMM_sub = nh.subscribe("/gmm_share", 1, gmmCallback);
	GMM_pub = nh.advertise<gmm_test::GMM>("/gmm_share",1);
	point_pub = nh.advertise<sensor_msgs::PointCloud2>("gmm_cloud_sample",1);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("gmm_cloud",1);
    ellipsoid_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("GMM_visual_1",1);
    cluster_centre_pub = nh.advertise<visualization_msgs::Marker>("GMM_visual_2",1);
    cloud_sub = nh.subscribe("/iris_0/camera/depth/points", 1, cloudCallback);
	position_sub = nh.subscribe("/uav0/mavros/local_position/pose", 1, placeCallback);
	headangle_sub = nh.subscribe("/iris/gimbal_yaw_angle", 1, headangleCallback);

    ros::Duration(0.1).sleep();
	ros::spin();
    return 0;
}

void gmmCallback(gmm_test::GMM gmm_msg){

	std::vector<Eigen::Vector3d> means;
    std::vector<Eigen::Matrix3d> var;
    std::vector<double> pi_vector;

	for(int i=0;i<gmm_msg.means.size()/3;i++)
	{
		Eigen::Vector3d tmp;
		tmp<<gmm_msg.means[i*3+0],gmm_msg.means[i*3+1],gmm_msg.means[i*3+2];
		means.push_back(tmp);

		Eigen::Matrix3d tmp_var;
		tmp_var<<gmm_msg.vars[i*9+0],gmm_msg.vars[i*9+1],gmm_msg.vars[i*9+2],
				gmm_msg.vars[i*9+3],gmm_msg.vars[i*9+4],gmm_msg.vars[i*9+5],
				gmm_msg.vars[i*9+6],gmm_msg.vars[i*9+7],gmm_msg.vars[i*9+8];
		var.push_back(tmp_var);

		double tmp_pi=gmm_msg.pi[i];
		pi_vector.push_back(tmp_pi);
	}

	std::vector<Eigen::Vector3d> point_sample;
	pcl::PointCloud<pcl::PointXYZ>::Ptr gvmcbMap_pc(new pcl::PointCloud<pcl::PointXYZ>());
	
	gvm::GMM gmm_sample;
	gmm_sample.init(pi_vector,means,var,means.size(),means[0]);
	point_sample = gmm_sample.GMMRandomSamplePoints(1000,2);//2为最佳


	for(int i = 0;i<point_sample.size();i++){
		pcl::PointXYZ vis_point;
		vis_point.x = point_sample[i](0);
		vis_point.y = point_sample[i](1);
		vis_point.z = point_sample[i](2);
		gvmcbMap_pc->points.push_back(vis_point);
	}
	sensor_msgs::PointCloud2 gvmcbMap_ros;
	pcl::toROSMsg(*gvmcbMap_pc, gvmcbMap_ros);
	gvmcbMap_ros.header.stamp = ros::Time::now();
	gvmcbMap_ros.header.frame_id = "iris_0/camera_link";
	point_pub.publish(gvmcbMap_ros);
	//std::cout<<point_sample.size()<<std::endl;


}

void headangleCallback(geometry_msgs::PoseStamped msg){
	double angle = msg.pose.position.x;
	double pi_headangle = 3.1415926;

    while(angle>=pi_headangle)
        angle-=2*pi_headangle;
    while(angle<-pi_headangle)
        angle+=2*pi_headangle;

	std::ostringstream str_iris;
	str_iris<<"iris_"<<AGENT_ID;
	if(msg.header.frame_id==str_iris.str())
	{
		currentHeadAngle=msg.pose.position.x;
	}

	//std::cout<<currentHeadAngle<<std::endl;

}

void placeCallback(geometry_msgs::PoseStamped msg){

    current_p(0) = msg.pose.position.x + initial_offset_x;
    current_p(1) = msg.pose.position.y + initial_offset_y;
    current_p(2) = msg.pose.position.z + initial_offset_z;

    current_q(0) = msg.pose.orientation.x;
    current_q(1) = msg.pose.orientation.y;   
    current_q(2) = msg.pose.orientation.z;
    current_q(3) = msg.pose.orientation.w;

}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{	

    ros::Time cloudcbstart=ros::Time::now();
	//std::cout<<1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *cloud_in);

    if(cloud_in->points.size()<=20) return;

	double motor_yaw = 0.0;
	//motor_yaw = currentHeadAngle;
	Eigen::Quaterniond  quad(1,0,0,0);
	quad.x() = current_q(0);
	quad.y() = current_q(1);
    quad.z() = current_q(2);
	quad.w() = current_q(3);
    Eigen::Matrix4d vehicle2ground = Eigen::Matrix4d::Identity();
	Eigen::Quaterniond  quad_sychronized = quad;
    vehicle2ground.block(0, 0, 3, 3) = Eigen::Matrix3d(quad_sychronized);
    vehicle2ground(0, 3) = current_p(0);
    vehicle2ground(1, 3) = current_p(1);
    vehicle2ground(2, 3) = current_p(2);


    //////////////////////////////////////////////STEP1//////////////////////////////////////////////
    //////////////////////////////////////////////直通滤波//////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud_in);
	pass.setFilterFieldName ("z");// 滤波字段设置为z轴方向
	pass.setFilterLimits (near_distance, far_distance);
	//pass.setFilterLimitsNegative (true); //默认为flase。true为过滤掉范围内的，flase为过滤掉范围外的
	pass.filter (*cloud_1);

	int number_sample;
	if(cloud_1->points.size()/10<4000)
		number_sample = cloud_1->points.size()/10;
	else
		number_sample = 4000;

	pcl::RandomSample<pcl::PointXYZ> rs;	//创建滤波器对象
	rs.setInputCloud(cloud_1);				//设置待滤波点云
	rs.setSample(number_sample);					//设置下采样点云的点数
	//rs.setSeed(1);						//设置随机函数种子点
	rs.filter(*cloud_1);					//执行下采样滤波，保存滤波结果于cloud_sub

	// // 创建环境
	// pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// // 创建滤波器
	// //std::cout<<R_filter<<std::endl;
	// outrem.setInputCloud(cloud_1);//设置输入点云
	// outrem.setRadiusSearch(R_filter);//设置在0.8的半径内找临近点
	// outrem.setMinNeighborsInRadius(N_filter);//设置查询点的邻近点集数小于2的删除
	// // 应用滤波器
	// outrem.filter (*cloud_1); 

    //////////////////////////////////////////////STEP2//////////////////////////////////////////////
    //////////////////////////////////////////////体素滤波//////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_1);
	static float res = grid_size;
	sor.setLeafSize(res, res, res);
	sor.setMinimumPointsNumberPerVoxel(N_filter);
	sor.filter(*cloud_2);
	//std::cout<<cloud_2->points.size()<<std::endl;

    //////////////////////////////////////////////STEP3//////////////////////////////////////////////
    //////////////////////////////////////////////点云格式转换与旋转//////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4d d435base2vehicle = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond d435base2vehicle_rotate(cos(motor_yaw/2.0),0,0,sin(motor_yaw/2.0));
    d435base2vehicle.block(0, 0, 3, 3) = Eigen::Matrix3d(d435base2vehicle_rotate);
    d435base2vehicle(0, 3) = 0.0;
    d435base2vehicle(1, 3) = 0.0;
    d435base2vehicle(2, 3) = 0.15;

    Eigen::Matrix4d d4352d435base = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond d4352d435base_rotate(0.5,-0.5,0.5,-0.5);
    d4352d435base.block(0, 0, 3, 3) = Eigen::Matrix3d(d4352d435base_rotate);
    d4352d435base(0, 3) = 0.0;
    d4352d435base(1, 3) = 0.0;
    d4352d435base(2, 3) = 0.0;
	Eigen::Matrix4d transform = vehicle2ground*d435base2vehicle*d4352d435base;  //d435 to ground


	//pcl::transformPointCloud(*cloud_2, *cloud_3, transform);
	cloud_3 = cloud_2;
	

    pass.setInputCloud (cloud_3);
    pass.setFilterFieldName ("z");
	if(current_p(1)-Z_boundary>0.2)
		pass.setFilterLimits (current_p(1)-Z_boundary, current_p(1)+Z_boundary);
	else
    	pass.setFilterLimits (0.2, current_p(1)+Z_boundary);
    pass.filter (*cloud_2);
    //std::cout<<cloud_2->points.size()<<std::endl;
    if(cloud_2->points.size() < 20) return;

	int count_0 = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNotAtedge(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(int i=0;i<(int)cloud_2->points.size();i++)
	{
		Eigen::Vector3d tmp_point(cloud_2->points[i].x,cloud_2->points[i].y,cloud_2->points[i].z);
		// if( (pow(tmp_point(0)-current_p(0),2.0)+
		//      pow(tmp_point(1)-current_p(1),2.0)+
		//      pow(tmp_point(2)-current_p(2),2.0)) <0.1 )
		// break;
		pcl::PointXYZRGB point;
		point.x=cloud_2->points[i].x;
		point.y=cloud_2->points[i].y;
		point.z=cloud_2->points[i].z;
		point.b=255;
		cloudNotAtedge->points.push_back(point);
		count_0++;
	}
	//std::cout<<count_0<<std::endl;
	if(count_0<20) return;

    ros::Time cloudmid=ros::Time::now();

    //////////////////////////////////////////////STEP4//////////////////////////////////////////////
    //////////////////////////////////////////////欧式聚类//////////////////////////////////////////////
	/// 创建kd树
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloudNotAtedge);
	/// 设置分割参数, 执行欧式聚类分割
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.3);	//设置近邻搜索的半径
	ec.setMinClusterSize(1);		//设置最小聚类点数
	ec.setMaxClusterSize(99999);	//设置最大聚类点数
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudNotAtedge);
	ec.extract(cluster_indices);

    if(cluster_indices.size()==0)return;


    //////////////////////////////////滤波后点云发送//////////////////////////////////////////////
	sensor_msgs::PointCloud2 cloud2_pub;
	pcl::toROSMsg(*cloud_2,cloud2_pub);//滤波后的点云转换为ROS消息
	cloud2_pub.header.frame_id = "iris_0/camera_link";
	cloud_pub.publish(cloud2_pub);

	
    //////////////////////////////////////////////STEP5//////////////////////////////////////////////
    ////////////////////////////聚类算法（K-means与EM的分层拟合）///////////////////////////
	////original
		
	std::vector<Eigen::Vector3d> means;
    std::vector<Eigen::Matrix3d> var;
    std::vector<double> pi_vector;
	std::vector<double> R_cover;
	int N_gau = 0;
	K_time = 0.0;
	E_time = 0.0;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<Eigen::Vector3d> point_cloud;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			Eigen::Vector3d	tmp(cloudNotAtedge->points[*pit].x,cloudNotAtedge->points[*pit].y,cloudNotAtedge->points[*pit].z);
			point_cloud.push_back(tmp);
		}
		gy::GMM_fit GMM;
		GMM.grid_size = grid_size;
		GMM.weight_L_f = 0.4;
		GMM.weight_node = 3000.0;
		GMM.weight_cover = 1000.0;
		GMM.max_number = max_number;
		GMM.EM_begin_number = EM_begin_number;
		GMM.point = point_cloud;
		GMM.GMM_fit_layer();
		
		for(int i = 0;i<GMM.miu.size();i++){
			if(abs(GMM.miu[i].sum()) <1e-5 ) break;
			//if(GMM.alpha[i] == 0||GMM.alpha[i] == 1)break;
			means.push_back(GMM.miu[i]);
			var.push_back(GMM.sigma[i]);
			pi_vector.push_back((double)1.0/GMM.miu.size());
			N_gau++;
		}
		R_cover.push_back(GMM.R_cover);
		//R_cover = GMM.R_cover;
		//R_cover = R_cover+0.15;
		//if(R_cover>1.0) R_cover = 1.0;
		//if(R_cover<0.16) R_cover = 0.8;
	}

	// for(int i  = 0; i<R_cover.size( );i++){
	// 	std::cout<<R_cover[i];
	// 	std::cout<<", ";
	// 	std::cout<<std::endl;
	// }
	// std::cout<<std::endl;

	double R_cover_tmp_mean = 0.0;
	int R_tmp_i = 0;
	for(int i = 0;i<R_cover.size();i++){
		if (R_cover[i]>0.5){
			R_cover_tmp_mean += R_cover[i];
			R_tmp_i++;
		}
	}
	R_cover_tmp_mean /= (double)R_tmp_i;
	//std::cout<<"R_cover_mean"<<"= "<<R_cover_tmp_mean<<std::endl;
		//std::cout<<means[means.size()-1]<<std::endl;
			// if(means[i].sum() == 0)
			// ROS_INFO("means");
			// if(pi_vector[i] == 0)
			// ROS_INFO("pi");
	

	///////////////////////////////////////////////////////////////////////////////
	/*
	if(test_mark_one_time == time_max_to_mean) return;
	std::cout<<"time_start = "<<test_mark_one_time+1<<std::endl;
	//std::cout<<"time_max_to_mean = "<<time_max_to_mean<<std::endl;
	//std::cout<<"time"<<std::endl;
	//std::cout<<"a"<<std::endl;
	double W1_begin    = 1;
	double W1_interval = 1;
	double W1_end      = 1;

	double W2_begin    = 0;
	double W2_interval = 100;
	double W2_end      = 2000;

	double W3_begin    = 0;
	double W3_interval = 1000;
	double W3_end      = 30000;

	for(double weight_L_f_ = W1_begin; weight_L_f_<W1_end+W1_interval; weight_L_f_ = weight_L_f_+W1_interval){
	for(double weight_node_ = W2_begin; weight_node_<W2_end+1; weight_node_ = weight_node_+W2_interval){
		for(double weight_cover_ = W3_begin; weight_cover_<W3_end+1; weight_cover_ = weight_cover_+W3_interval){
			//std::cout<<"b"<<std::endl;
			std::vector<Eigen::Vector3d> means;
    		std::vector<Eigen::Matrix3d> var;
    		std::vector<double> pi_vector;
			std::vector<double> R_cover;
			int N_gau = 0;
			K_time = 0.0;
			E_time = 0.0;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
			{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
			std::vector<Eigen::Vector3d> point_cloud;
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			{
				Eigen::Vector3d	tmp(cloudNotAtedge->points[*pit].x,cloudNotAtedge->points[*pit].y,cloudNotAtedge->points[*pit].z);
				point_cloud.push_back(tmp);
			}
			gy::GMM_fit GMM;
			GMM.weight_L_f = weight_L_f_;
			GMM.weight_node = weight_node_;
			GMM.weight_cover = weight_cover_;
			GMM.max_number = max_number;
			GMM.EM_begin_number = EM_begin_number;
			GMM.point = point_cloud;
			GMM.GMM_fit_layer();
		
			for(int i = 0;i<GMM.miu.size();i++){
				if(abs(GMM.miu[i].sum()) <1e-5 ) break;
				if(GMM.alpha[i] == 0||GMM.alpha[i] == 1)break;
				means.push_back(GMM.miu[i]);
				var.push_back(GMM.sigma[i]);
				pi_vector.push_back(GMM.alpha[i]);
				N_gau++;
			}
				R_cover.push_back(GMM.R_cover);
			}
			//std::cout<<"c"<<std::endl;
			// for(int i  = 0; i<R_cover.size( );i++){
			// 	std::cout<<R_cover[i];
			// 	std::cout<<", ";
			// 	std::cout<<std::endl;
			// }
			// std::cout<<std::endl;

			double R_cover_tmp_mean = 0.0;
			int R_tmp_i = 0;
			for(int i = 0;i<R_cover.size();i++){
				if (R_cover[i]>0.5){
					R_cover_tmp_mean += R_cover[i];
					R_tmp_i++;
				}
			}
			R_cover_tmp_mean /= (double)R_tmp_i;

			// std::cout<<"W1"  <<" = "<<weight_L_f_;
			// std::cout<<"; W2"<<" = "<<weight_node_;
			// std::cout<<"; W3"<<" = "<<weight_cover_;
			// std::cout<<"; R_cover_mean" <<" = "<<R_cover_tmp_mean<<std::endl;
			//std::cout<<"test_mark_one_time = "<<test_mark_one_time<<std::endl;
			//std::cout<<"N_gau"<<N_gau<<std::endl;
			N_all[test_mark_one_time].push_back((double)N_gau);
			recover_all[test_mark_one_time].push_back(R_cover_tmp_mean);
			//std::cout<<"size_mark = "<<test_mark_one_time<<std::endl;
			//std::cout<<"e"<<std::endl;
			//std::cout<<means[means.size()-1]<<std::endl;
			// if(means[i].sum() == 0)
			// ROS_INFO("means");
			// if(pi_vector[i] == 0)
			// ROS_INFO("pi");
			visual_ellipsoid(means,var,2.0,0);
		}
	}
	}
	//std::cout<<"time_end = "<<test_mark_one_time+1<<std::endl<<std::endl;
	//将所有的数据求平均，并打印出来
	// for(int i = 0; i< recover_all.size(); i++){
	// 	std::cout<<recover_all[i].size()<<std::endl;
	// }
	if(test_mark_one_time == time_max_to_mean-1){
		//std::cout<<"t"<<recover_all[test_mark_one_time].size()<<std::endl;
		std::vector<double> recover_all_mean(recover_all[test_mark_one_time].size());
		std::vector<double> N_all_mean(N_all[test_mark_one_time].size());
		for (int i = 0; i<recover_all[test_mark_one_time].size();i++){
			double recover_now = 0.0;
			double N_now = 0.0;
			for(int j = 0; j<time_max_to_mean; j++){
				recover_now += recover_all[j][i];
				N_now += N_all[j][i];
			}
			recover_now = recover_now/(double)time_max_to_mean;
			N_now = N_now/(double)time_max_to_mean;
			recover_all_mean[i] = recover_now;
			N_all_mean[i] = N_now;
		}
		// for(int i = 0;i<recover_all_mean.size();i++){
		// 	std::cout<<recover_all_mean[i]<<std::endl;
		// 	////
		// }
		//std::cout<<"1"<<std::endl;
		int i_tmp_visual = 0;
		//std::cout<<"; R_cover_mean" <<": "<<recover_all_mean.size()<<std::endl;
		for(double weight_L_f_ = W1_begin; weight_L_f_<W1_end+W1_interval; weight_L_f_ = weight_L_f_+W1_interval){
			for(double weight_node_ = W2_begin; weight_node_<W2_end+1; weight_node_ = weight_node_+W2_interval){
				for(double weight_cover_ = W3_begin; weight_cover_<W3_end+1; weight_cover_ = weight_cover_+W3_interval){
				std::cout<<"W1"  <<": "<<weight_L_f_;
				std::cout<<"; W2"<<": "<<weight_node_;
				std::cout<<"; W3"<<": "<<weight_cover_;
				std::cout<<"; N"<<": "<<N_all_mean[i_tmp_visual];
				std::cout<<"; R_cover_mean" <<": "<<recover_all_mean[i_tmp_visual]<<std::endl;
				i_tmp_visual++;
				}
			}
		}
	}
	test_mark_one_time ++;
	*/
	////////end/////////////////////////////////////////////////////////////////////

	// std::cout<<means.size()<<std::endl;
	// std::cout<<means[0]<<std::endl;
	// std::cout<<var[0]<<std::endl;

	//ros::Duration(2).sleep();

    ros::Time cloudend =ros::Time::now();	
	//std::cout<<cloudmid<<std::endl;
	// std::cout<<"Filter:"<<(cloudmid-cloudcbstart).toSec()*1000;
	// std::cout<<" K:"<<K_time*1000;
	// std::cout<<" E:"<<E_time*1000;
	// std::cout<<" N:"<<N_gau<<" P:"<<count_0<<std::endl;
	//std::cout<<"F:"<<(cloudend-cloudmid).toSec()*1000<<std::endl;

    // std::ofstream ofs;  
    // ofs.open("/home/hat/GMM_test_ws/GMM.txt",std::ios::out);
	// ofs<<"Filter:"<<(cloudmid-cloudcbstart).toSec()*1000;
	// ofs<<" K:"<<K_time*1000;
	// ofs<<" E:"<<E_time*1000;
	// ofs<<" N:"<<N_gau<<" P:"<<count_0<<std::endl;
    // ofs.close();

	// double R_cover_tmp_mean = 0.0;
	// int R_tmp_i = 0;
	// for(int i = 0;i<R_cover.size();i++){
	// 	if (R_cover[i]>0.5){
	// 		R_cover_tmp_mean += R_cover[i];
	// 		R_tmp_i++;
	// 	}
	// }
	// R_cover_tmp_mean /= (double)R_cover.size();
	double now_fps = pow((cloudend-cloudcbstart).toSec(),-1);
	// if(R_cover_tmp_mean < min_R_cover) min_R_cover = R_cover_tmp_mean;
	// if(R_cover_tmp_mean > max_R_cover)max_R_cover = R_cover_tmp_mean;
	// mean_R_cover = (count_fps/(count_fps + 1.0)*mean_R_cover) + (1/(count_fps + 1.0)*R_cover_tmp_mean);
	// count_fps = count_fps + 1.0;	

	if(now_fps < min_fps) min_fps = now_fps;
	if(now_fps > max_fps) max_fps = now_fps;
	mean_fps = (count_fps/(count_fps + 1.0)*mean_fps) + (1/(count_fps + 1.0)*now_fps);

	// ros::Time cloud_test1 =ros::Time::now();
	// for(int i = 0;i<1000;i++){
	// 	for(int j = 0;j<1000;j++){
	// 		for(int k = 0;k<10000;k++){
	// 			100*100;
	// 		}
	// 	}
	// }
	// ros::Time cloud_test2 =ros::Time::now();
	// std::cout<<(cloud_test2-cloud_test1)*1000<<std::endl;
	
	std::cout<<"NOW_fps="<<pow((cloudend-cloudcbstart).toSec(),-1)<<"\t; whole_time = "
		<<std::setprecision(3)<<(cloudend-cloudcbstart)*1000<<" ms"<<"\t; fit_time = "<<std::setprecision(3)<<(cloudend-cloudmid)*1000<<" ms "<<std::endl;
	//std::cout<<"N = "<<N_gau*(3+9+1)*8<<"kb;   "<<"Point_cloud = "<<count_0*3*8<<"kb;"<<std::endl;
    //ROS_INFO("Total_time = %f s, Fit_time =  %f s, now_fps = %f , mean = %f min = %f max = %f ",(cloudend-cloudcbstart).toSec(),(cloudend-cloudmid).toSec(),pow((cloudend-cloudcbstart).toSec(),-1),mean_fps,min_fps,max_fps);
	//ROS_INFO("now_fps = %f , mean = %f min = %f max = %f ",pow((cloudend-cloudcbstart).toSec(),-1),mean_fps,min_fps,max_fps);
    //ROS_INFO("now_R = %f , mean = %f min = %f max = %f ",R_cover,mean_R_cover,min_R_cover,max_R_cover);
	//std::cout<<"########"<<std::endl;
	//std::cout<<"N="<<N_gau<<std::endl;
	//std::cout<<"now_fps ="<<(int)now_fps<<std::endl;
	//std::cout<<"mean_fps ="<<(int)mean_fps<<", min_fps ="<<(int)min_fps<<", max_fps ="<<(int)max_fps<<std::endl;
	//std::cout<<"now_R ="<<(int)R_cover<<std::endl;
	//std::cout<<"mean_R ="<<(float)mean_R_cover<<", min_R ="<<(float)min_R_cover<<", max_R ="<<(float)max_R_cover<<std::endl;
	//std::cout<<"########"<<std::endl<<std::endl;


	
    //////////////////////////////////////////////STEP6//////////////////////////////////////////////
    /////////////////////////////////////////////拟合结果可视化///////////////////////////////////////
    if(means.size() == 0)return;
	// for(int  i = 0;i<means.size();i++){
	// 	means[i](1) += 3;
	// }

    // sensor_msgs::PointCloud2 cloud2_pub;
	// pcl::toROSMsg(*cloud_2,cloud2_pub);//滤波后的点云转换为ROS消息
	// cloud2_pub.header.frame_id = "iris_0/camera_link";
	// cloud_pub.publish(cloud2_pub);

    visual_ellipsoid(means,var,2.0,0);
    

    //////////////////////////////////////////////STEP7//////////////////////////////////////////////
    /////////////////////////////////////////////发送GMM消息//////////////////////////////////

	gmm_test::GMM gmm_msg;
	
	for(int i=0;i<means.size();i++)
	{
		gmm_msg.pi.push_back(pi_vector[i]);

		gmm_msg.means.push_back(means[i](0));
		gmm_msg.means.push_back(means[i](1));
		gmm_msg.means.push_back(means[i](2));

		for(int j=0;j<3;j++)
		{
			for(int k=0;k<3;k++)
			{
				gmm_msg.vars.push_back(var[i](j,k));
			}
		}
	}

	GMM_pub.publish(gmm_msg);
	

}

//int mark__g = 0;
void visual_ellipsoid(std::vector<Eigen::Vector3d> means,std::vector<Eigen::Matrix3d> var,double n_sigma,int id)
{	
	//if (mark__g == 1)
	// return;
	// mark__g =1 ;

//////////////////////visual_for one frame
for (int i = 0; i<80;i++){
	Eigen::Vector3d mean_ee;
	mean_ee<<0,0,0;
	Eigen::Matrix3d var_ee;
	var_ee<<0,0,0,0,0,0,0,0,0;
	means.push_back(mean_ee);
	var.push_back(var_ee);
}
id_g = 0;
//////////////////////////////////////////

visualization_msgs::MarkerArray marker_array;

int cluster_num=means.size();
// ROS_INFO("means var size %ld %ld",means.size(),var.size());

for(int i=0;i<cluster_num;i++)
{
	for(int j=0;j<3;j++)
		{
		for(int k=0;k<3;k++)
			{
			if(fabs(var[i](j,k))<1e-5)
				{
				var[i](j,k)=0;
				}
			}
		}

Eigen::EigenSolver<Eigen::Matrix3d> es(var[i]);

Eigen::Matrix3d value_martrix = es.pseudoEigenvalueMatrix();
Eigen::Matrix3d vector_martrix = es.pseudoEigenvectors();

vector_martrix.col(2)=vector_martrix.col(0).cross(vector_martrix.col(1));
// std::cout<<"var: "<<var[i]<<endl;

// std::cout<<"value_martrix: \n"<<value_martrix<<endl;
// std::cout<<"vector_martrix: \n"<<vector_martrix<<endl;
// std::cout<<"means[i]: "<<means[i]<<endl;


double var0=pow(value_martrix(0,0),0.5)*n_sigma;
double var1=pow(value_martrix(1,1),0.5)*n_sigma;
double var2=pow(value_martrix(2,2),0.5)*n_sigma;
// ROS_INFO("i: %d var: %f %f %f, means: %f %f %f",i,var0,var1,var2,means[i](0),means[i](1),means[i](2));

visualization_msgs::Marker marker;

marker.header.frame_id = "iris_0/camera_link";//Gazebo
//marker.header.frame_id = "camera_rgb_optical_frame";//ETH
//marker.header.frame_id = "/openni_rgb_optical_frame";//TUM
marker.header.stamp = ros::Time();
marker.id = id_g+(AGENT_ID*1000000);
id_g++;
if(id_g>1000000) id_g = 0;
//std::cout<<id_g*(AGENT_ID+1)<<std::endl;
marker.type = visualization_msgs::Marker::SPHERE;
marker.action = visualization_msgs::Marker::ADD;


if(mean_fps == 0) mean_fps = 10;
marker.lifetime = ros::Duration((pow(mean_fps,-1)>0.01)?pow(mean_fps,-1):0.01); //保留显示
marker.lifetime = ros::Duration(300);


marker.pose.position.x = means[i](0);
marker.pose.position.y = means[i](1);
marker.pose.position.z = means[i](2);

Eigen::Quaterniond quaternion(vector_martrix);
quaternion.normalize();
marker.pose.orientation.x = quaternion.x();
marker.pose.orientation.y = quaternion.y();
marker.pose.orientation.z = quaternion.z();
marker.pose.orientation.w = quaternion.w();

marker.scale.x = var0*2;
marker.scale.y = var1*2;
marker.scale.z = var2*2;

marker.color.a = 0.5; // Don't forget to set the alpha!
marker.color.r = 0.0;
marker.color.g = 0.0;
marker.color.b = 0.0;

marker_array.markers.push_back(marker);

}

ellipsoid_marker_array_pub.publish(marker_array);

// visulization of cluster centre
visualization_msgs::Marker point_marker;

point_marker.id = 333;
point_marker.header.frame_id = "iris_0/camera_link";
point_marker.header.stamp = ros::Time::now();
point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
point_marker.action = visualization_msgs::Marker::ADD;
point_marker.ns = "lines_and_points";

point_marker.scale.x = 0.1;
point_marker.scale.y = 0.1;
point_marker.scale.z = 0.1;

point_marker.lifetime = ros::Duration(1);
//point_marker.lifetime = ros::Duration(pow(mean_fps,-1));

point_marker.color.r=1;
point_marker.color.g=0;
point_marker.color.b=0;
point_marker.color.a=1.0;

for(int i=0;i<means.size();i++)
{
geometry_msgs::Point point;
point.x = means[i](0);
point.y = means[i](1);
point.z = means[i](2);
point_marker.points.push_back(point);
}

cluster_centre_pub.publish(point_marker);

}