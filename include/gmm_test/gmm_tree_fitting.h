#ifndef EM_fit_gy
#define EM_fit_gy

#include <iostream>
#include <Eigen/Dense>
#include <ctime>
#include <math.h>
#include <numeric>
#include <algorithm>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "gmm_test/gmm_sample.h"

ros::Subscriber cloud_sub;
ros::Publisher ellipsoid_marker_array_pub;
ros::Publisher cluster_centre_pub;
ros::Publisher cloud_pub;
ros::Publisher point_pub;
ros::Subscriber position_sub;
ros::Subscriber headangle_sub;
ros::Publisher GMM_pub;
ros::Subscriber GMM_sub;
int AGENT_ID = 0;
double initial_offset_x = 0;
double initial_offset_y = 0;
double initial_offset_z = 0;
double currentHeadAngle = 0;
Eigen::Vector3d current_p;
Eigen::Vector4d current_q;
double near_distance = 0.0;
double far_distance = 5.0;
double Z_boundary = 1.5;
double grid_size = 0.2;
double R_filter = 0.8;
int N_filter = 100;
int EM_begin_number = 150;
int max_number = 100;
double max_fps = 0;
double min_fps = 9999;
double max_R_cover = 0.0;
double min_R_cover = 1.0;
double mean_R_cover = 0.0;
double mean_fps = 0;
double count_fps = 0;
int id_g =  0;
int test_mark_one_time = 0;
int time_max_to_mean = 50;
std::vector<std::vector<double> > recover_all(time_max_to_mean);
std::vector<std::vector<double> > N_all(time_max_to_mean);
void visual_ellipsoid(std::vector<Eigen::Vector3d> means,std::vector<Eigen::Matrix3d> var,double n_sigma,int id);

std::vector<std::vector<Eigen::Vector3d> > means_visual;
std::vector<std::vector<Eigen::Matrix3d> > var_visual;

double K_time = 0.0;
double E_time = 0.0;



std::vector<Eigen::Vector3d> point_sample_(std::vector<Eigen::Vector3d> means,
							std::vector<Eigen::Matrix3d> var,std::vector<double> pi,int number,double n_sigma);
double getMahalanobisDistance(Eigen::Vector3d point,Eigen::Vector3d means,Eigen::Matrix3d var);
Eigen::Vector3d cover_rate_cal(std::vector<Eigen::Vector3d> point,std::vector<double> weight1,
																		std::vector<double> weight2,Eigen::Vector3d mean1,Eigen::Vector3d mean2,
																		Eigen::Matrix3d var1,Eigen::Matrix3d var2, double n_sigma,double grid_size);
double GMM_V_cal(Eigen::Matrix3d var, double n_sigma,double grid_size);

namespace gy {


	class point_banch {
	public:
		std::vector<Eigen::Vector3d> point;//点云
		std::vector<double> weight;//每个点的权重
		bool mark = 0;//该点云是否需要被再次分解
		double BIC_value = 9999999999;//该点云的BIC数值

		//GMM簇参数
		double R_cover = 0;
		double alpha_k;
		Eigen::Vector3d miu_k;
		Eigen::Matrix3d sigma_k;
	};

	class return_parameter {
	public:
		std::vector<bool> divide;
		std::vector<double> weight_1;
		std::vector<double> weight_2;
		double alpha_k_1;
		double alpha_k_2;
		Eigen::Vector3d miu_k_1;
		Eigen::Vector3d miu_k_2;
		double BIC;
		double BIC_1;
		double BIC_2;
		double R_cover;
		double R_cover_1;
		double R_cover_2;
		Eigen::Matrix3d sigma_1;
		Eigen::Matrix3d sigma_2;

	};

	
	class GMM_fit {
	public:
		std::vector<Eigen::Vector3d>  point;
		std::vector<Eigen::Vector3d>  miu;
		std::vector<Eigen::Matrix3d>  sigma;
		std::vector<double> alpha;
		double R_cover;
		int max_number = 2000;//////////////////////////////
		int EM_begin_number = 150;
		double n_sigma = 2;
		double grid_size = 0.1;

		double weight_L_f = 1.0;
		double weight_node = 150.0;
		double weight_cover = 100;

	public:
		void GMM_fit_layer() {
			//初始化点云二叉树
			std::vector<point_banch> point_tree;
			//std::cout<<"1";
			//生成一个点云二叉树树杈的集合
			//初始化该树杈集合
			point_banch tmp_banch;
			tmp_banch.point = point;
			std::vector<double> weight_tmp(point.size(),1);
			tmp_banch.weight = weight_tmp;

			//存入二叉树的树干
			point_tree.push_back(tmp_banch);
			//std::cout << point_tree.size()<<std::endl;
			//开始二叉树分解
			while (tree_all_fitted(point_tree) == 0) {
                if(point_tree.size()>1000)break;
				for (int i = 0; i < point_tree.size(); i++) {
					if (point_tree[i].mark == 0) {//若该树杈等待被分解，则开始分解
						if (point_tree[i].point.size()<20) {//如果一个集合点太少，则直接判为不需要拟合
							point_tree[i].mark = 1;
							break;
						}
						return_parameter tmp_parameter;
						//先由Kmeans算法初始化分解
						ros::Time fitting_start=ros::Time::now();
						tmp_parameter = K_means_cal(point_tree[i].point);
						//若点个数足够小，则由EM算法继续拟合
						if (point_tree[i].point.size() < EM_begin_number) {
							tmp_parameter = EM_cal(point_tree[i].point,tmp_parameter);
						}
						ros::Time fitting_end=ros::Time::now();
						if (point_tree[i].point.size() < EM_begin_number){
							E_time = E_time + (fitting_end-fitting_start).toSec();
						}
						else{
							K_time = K_time + (fitting_end-fitting_start).toSec();
						}
						//若BIC满足条件，则不进行进一步分裂////&& point_tree[i].point.size() < max_number////
						//std::cout<<tmp_parameter.BIC<<"    "<<point_tree[i].BIC_value<<std::endl;
						if (tmp_parameter.BIC >= point_tree[i].BIC_value && point_tree[i].point.size() < max_number 
							|| isnan(tmp_parameter.BIC) || isnan(point_tree[i].BIC_value)) {
							point_tree[i].mark = 1;
							//std::cout<<"stop!"<<std::endl;
							break;
						}
						//std::cout<<"continue!"<<std::endl;
						point_banch point_banch_tmp_0;
						point_banch point_banch_tmp_1;

						for (int j = 0; j < point_tree[i].point.size(); j++) {
							if (tmp_parameter.weight_1[j] > tmp_parameter.weight_2[j]) {
								point_banch_tmp_0.point.push_back(point_tree[i].point[j]);
								//point_banch_tmp_0.weight.push_back(point_tree[i].weight[j] * tmp_parameter.weight_1[j]);
							}
							else {
								point_banch_tmp_1.point.push_back(point_tree[i].point[j]);
								//point_banch_tmp_1.weight.push_back(point_tree[i].weight[j] * tmp_parameter.weight_2[j]);
							}
						}
						point_banch_tmp_0.R_cover = tmp_parameter.R_cover_1;
						point_banch_tmp_1.R_cover = tmp_parameter.R_cover_2;
						point_banch_tmp_0.BIC_value = tmp_parameter.BIC_1;
						point_banch_tmp_1.BIC_value = tmp_parameter.BIC_2;
						//point_banch_tmp_0.alpha_k = tmp_parameter.alpha_k_1 * point_tree[i].alpha_k;
						//point_banch_tmp_1.alpha_k = tmp_parameter.alpha_k_2 * point_tree[i].alpha_k;
						point_banch_tmp_0.miu_k = tmp_parameter.miu_k_1;
						point_banch_tmp_1.miu_k = tmp_parameter.miu_k_2;
						point_banch_tmp_0.sigma_k = tmp_parameter.sigma_1;
						point_banch_tmp_1.sigma_k = tmp_parameter.sigma_2;

						point_tree.erase(point_tree.begin() + i);
						point_tree.push_back(point_banch_tmp_0);
						point_tree.push_back(point_banch_tmp_1);
					
						break;
					}
				}
				//////////////////////存储可视化
				// std::vector<Eigen::Vector3d> means_tmp;
				// std::vector<Eigen::Matrix3d> var_tmp;
				// for (int i = 0; i < point_tree.size(); i++) {
				// 	if(abs(point_tree[i].miu_k.sum()) <1e-5 ) break;
				// 	if(point_tree[i].miu_k(0)*point_tree[i].miu_k(1)*point_tree[i].miu_k(2) == 0 ) break;
				// 	if(abs(point_tree[i].sigma_k.sum()) <1e-5 ) break;
				// 	means_tmp.push_back(point_tree[i].miu_k);
				// 	var_tmp.push_back(point_tree[i].sigma_k);
				// }
				// if(means_visual.size()==0){
				// 	means_visual.push_back(means_tmp);
				// 	var_visual.push_back(var_tmp);
				// }
				// else if(means_visual[means_visual.size()-1].size() <  means_tmp.size()){
				// 	means_visual.push_back(means_tmp);
				// 	var_visual.push_back(var_tmp);
				// }

				/////////////////逐级可视化//////////
				// std::vector<Eigen::Vector3d> means;
				// std::vector<Eigen::Matrix3d> var;
				// for (int i = 0; i < point_tree.size(); i++) {
				// 	if(abs(point_tree[i].miu_k.sum()) <1e-5 ) break;
				// 	if(abs(point_tree[i].sigma_k.sum()) <1e-5 ) break;
				// 	means.push_back(point_tree[i].miu_k);
				// 	var.push_back(point_tree[i].sigma_k);
				// }
				// if(means.size()>0){
				// visual_ellipsoid(means,var,2.3,0);
				// ros::Duration(0.2).sleep();
				// }
				// /////////////////
			}

			R_cover = 0;
			double e_n = 0.0;
			for (int i = 0; i < point_tree.size(); i++) {
				if(point_tree[i].sigma_k.sum() == 0) break;
				if(point_tree[i].miu_k.sum() == 0) break;
				
				R_cover += point_tree[i].R_cover;
				e_n = e_n+1.0;
				//alpha.push_back((double)point_tree[i].point.size()/(double)point.size());
				miu.push_back(point_tree[i].miu_k);
				sigma.push_back(point_tree[i].sigma_k);
			}
			R_cover = R_cover/e_n;


			// //////////////////////////////存储可视化2
			// ros::Duration(1.5).sleep();
			// for(int i =0;i<means_visual.size();i++){
			// 	visual_ellipsoid(means_visual[i],var_visual[i],2.3,0);
			// 	ros::Duration(0.3).sleep();
			// }
			// means_visual.clear();
			// var_visual.clear();
			// ////////////////////////////////////////////////
		};

		int tree_all_fitted(std::vector<point_banch> point_tree) {
			for (int i = 0; i < point_tree.size(); i++) {
				if (point_tree[i].mark == 0) return 0;
			}
			return 1;

		};


		return_parameter K_means_cal(std::vector<Eigen::Vector3d> point) {
			return_parameter tmp_parameter;
			std::vector<double> alpha_cal(2);
			std::vector<Eigen::Vector3d> miu_cal(2);
			std::vector<Eigen::Matrix3d> sigma_cal(2);
			
			//std::cout<<"mark 0"<<std::endl;

			int N = point.size();						
			std::vector<bool> divide(N);							
			std::vector<Eigen::Vector2d> dist(N);							
			std::vector<Eigen::Vector2d> gamma_cal(N);							
				
			miu_cal[0] = point[1];							
			miu_cal[1] = point[5];							

			double dist_sum = 100;							
			double dist_sum_last = 0;							
			int step = 0;							
			double count_1 = 0.0;							
			double count_2 = 0.0;							
			while (fabs(dist_sum - dist_sum_last) > 0.01) {
				step++;
				if (step > 100) break;
				dist_sum_last = dist_sum;
				dist_sum = 0;

				count_1 = 0.0;
				count_2 = 0.0;
				for (int i = 0; i < N; i++) {
					dist[i](0) = pow((point[i] - miu_cal[0]).dot(point[i] - miu_cal[0]), 0.5);
					dist[i](1) = pow((point[i] - miu_cal[1]).dot(point[i] - miu_cal[1]), 0.5);
					//std::cout << "dist = " << dist[i](0) << ", " << dist[i](1)<<std::endl;
					if (dist[i](0) > dist[i](1)) {
						divide[i] = 1;
						dist_sum += dist[i](1);
						count_2 += 1.0;
					}
					else {
						divide[i] = 0;
						dist_sum += dist[i](0);
						count_1 += 1.0;
					}
				}
				//std::cout << "count1 = " << count_1 << std::endl;
				//std::cout << "count2 = " << count_2 << std::endl;
				//std::cout << miu_cal[0] << std::endl;
				//std::cout << miu_cal[1] << std::endl;
				miu_cal[0] << 0.0, 0.0, 0.0;
				miu_cal[1] << 0.0, 0.0, 0.0;
				for (int i = 0; i < N; i++) {
					if (divide[i] == 0) {
						miu_cal[0] += point[i];
					}
					else {
						miu_cal[1] += point[i];
					}
				}
                if(count_1 == 0)count_1 = 1;
                if(count_2 == 0)count_2 = 1;
				miu_cal[0] /= count_1;
				miu_cal[1] /= count_2;
				//std::cout<<"dist_sum = " << dist_sum << std::endl;
			}
			//std::cout<<"mark 2"<<std::endl;
			//std::cout << miu_cal[0] << std::endl;
			//std::cout << miu_cal[1] << std::endl;
			sigma_cal[0] = Eigen::MatrixXd::Constant(3, 3, 0);
			sigma_cal[1] = Eigen::MatrixXd::Constant(3, 3, 0);
			for (int i = 0; i < N; i++) {
				if (divide[i] == 0) {
					sigma_cal[0] += (point[i] - miu_cal[0]) * (point[i] - miu_cal[0]).transpose();
				}
				else {
					sigma_cal[1] += (point[i] - miu_cal[1]) * (point[i] - miu_cal[1]).transpose();
				}
			}
			//std::cout<<"mark 3"<<std::endl;

			sigma_cal[0] /= count_1;
			sigma_cal[1] /= count_2;
			double alpha_1 = count_1 / (count_1 + count_2);
			double alpha_2 = count_2 / (count_1 + count_2);

			for (int i = 0; i < N; i++) {
				gamma_cal[i](0) = alpha_1 * GaussianP(point[i], miu_cal[0], sigma_cal[0]);
				gamma_cal[i](1) = alpha_2 * GaussianP(point[i], miu_cal[1], sigma_cal[1]);
			}

			double L_k1 = 0.0;
			double L_1 = 0.0;
			double L_2 = 0.0;
			double temp_gamma_sum;
			for (int n = 0; n < N; n++) {
				temp_gamma_sum = gamma_cal[n].sum();
				L_k1 += log(temp_gamma_sum);
				if(gamma_cal[n](0) > gamma_cal[n](1))
					L_1 += log(gamma_cal[n](0));
				else
					L_2 += log(gamma_cal[n](1));
				gamma_cal[n] /= temp_gamma_sum;
				if (gamma_cal[n](0) > 0.8) {
					gamma_cal[n](0) = 1;
					gamma_cal[n](1) = 0;
				}
				if (gamma_cal[n](1) > 0.8) {
					gamma_cal[n](1) = 1;
					gamma_cal[n](0) = 0;
				}
			}
			//std::cout<<"mark 4"<<std::endl;

			std::vector<double> weight_1(N);
			std::vector<double> weight_2(N);

			for (int i = 0; i < N; i++) {
				weight_1[i] = gamma_cal[i](0);
				weight_2[i] = gamma_cal[i](1);
			}
			
			// std::cout << "step = " << step;
			// std::cout << "BIC = " << -2 * L_k1 + 2 * log(N) << std::endl;
			// std::cout << "miu_1 = " << miu_cal[0] << std::endl;
			// std::cout << "miu_2 = " << miu_cal[1] << std::endl;
			// std::cout << "sigma_1 = " << sigma_cal[0] << std::endl;
			// std::cout << "sigma_2 = " << sigma_cal[1] << std::endl;
			//for (int i = 0; i < N; i++) {
			//	std::cout << "weight = " << weight_1[i]<<", "<< weight_2[i] << std::endl;
			//}
			//std::cout<<"mark 5"<<std::endl;
			Eigen::Vector3d temp_rate;
			temp_rate = cover_rate_cal(point,weight_1,weight_2,miu_cal[0],miu_cal[1],sigma_cal[0],sigma_cal[1],n_sigma,grid_size);

			tmp_parameter.alpha_k_1 = alpha_1;
			tmp_parameter.alpha_k_2 = alpha_2;
			tmp_parameter.R_cover = temp_rate(0);
			tmp_parameter.R_cover_1 =temp_rate(1);
			tmp_parameter.R_cover_2 = temp_rate(2);
			// tmp_parameter.BIC = -weight_L_f*L_k1 + 2*weight_node* log(N) - weight_cover*temp_rate(0);
			// tmp_parameter.BIC_1 = -weight_L_f*L_1 + 1*weight_node* log(count_1)- weight_cover*temp_rate(1);
			// tmp_parameter.BIC_2 = -weight_L_f*L_2 + 1*weight_node* log(count_2) - weight_cover*temp_rate(2);
			// if(isnan(L_k1)) L_k1 = 0;
			// if(isnan(L_1))  L_1 = 0;
			// if(isnan(L_2))  L_2 = 0;
			tmp_parameter.BIC = -weight_L_f*L_k1 + 2.0*weight_node/(double)N - weight_cover*temp_rate(0);
			//std::cout<<"L_k1="<<L_k1<<", 2/N="<<2.0/(double)N<<", cover="<<temp_rate(0)<<std::endl;
			tmp_parameter.BIC_1 = -weight_L_f*L_1 + 1.0*weight_node/(double)count_1- weight_cover*temp_rate(1);
			tmp_parameter.BIC_2 = -weight_L_f*L_2 + 1*weight_node/(double)count_2 - weight_cover*temp_rate(2);
			tmp_parameter.divide = divide;
			tmp_parameter.miu_k_1 = miu_cal[0];
			tmp_parameter.miu_k_2 = miu_cal[1];
			tmp_parameter.sigma_1 = sigma_cal[0];
			tmp_parameter.sigma_2 = sigma_cal[1];
			tmp_parameter.weight_1 = weight_1;
			tmp_parameter.weight_2 = weight_2;
			//std::cout<<"mark 6"<<std::endl;
			return tmp_parameter;

		};

		return_parameter EM_cal(std::vector<Eigen::Vector3d> point, return_parameter tmp_parameter) {
			return_parameter parameter;
			int N = point.size();
			double L_k1 = 100;
			double L_k0 = 1;

			double L_1 = 0.0;
			double L_2 = 0.0;

			// double alpha_1 = tmp_parameter.alpha_k_1;
			// double alpha_2 = tmp_parameter.alpha_k_2;
			double alpha_1 = 1;
			double alpha_2 = 1;
			Eigen::Vector3d miu_1 = tmp_parameter.miu_k_1;
			Eigen::Vector3d miu_2 = tmp_parameter.miu_k_2;
			Eigen::Matrix3d sigma_1 = tmp_parameter.sigma_1;
			Eigen::Matrix3d sigma_2 = tmp_parameter.sigma_2;
			std::vector <double> weight_1(N);
			std::vector <double> weight_2(N);

			std::vector <Eigen::Vector2d> gamma_cal(N);
 
			int step = 0;

			while (fabs(L_k1) / fabs(L_k0) > 1.005) {
				step++;
                if (step > 100) break;
				L_k0 = L_k1;
				for (int i = 0; i < N; i++) {
					gamma_cal[i](0) = alpha_1 * GaussianP(point[i], miu_1, sigma_1);
					gamma_cal[i](1) = alpha_2 * GaussianP(point[i], miu_2, sigma_2);
				}
				L_k1 = 0.0;
				L_1 = 0.0;
				L_2 = 0.0;
				double temp_gamma_sum;
				for (int n = 0; n < N; n++) {
					temp_gamma_sum = gamma_cal[n].sum();
					L_k1 += log(temp_gamma_sum);
					if(gamma_cal[n](0) > gamma_cal[n](1))
						L_1 += log(gamma_cal[n](0));
					else
						L_2 += log(gamma_cal[n](1));
					gamma_cal[n] /= temp_gamma_sum;
				}

				std::vector<double> t_0(2);
				std::vector<Eigen::Vector3d> t_1(2);
				std::vector<Eigen::Matrix3d> t_2(2);

				std::vector<double> T_0(2);
				std::vector<Eigen::Vector3d> T_1(2);
				std::vector<Eigen::Matrix3d> T_2(2);

				for (int k = 0; k < 2; k++) {
					T_0[k] = 0.0;
					T_1[k] = Eigen::MatrixXd::Constant(3, 1, 0);
					T_2[k] = Eigen::MatrixXd::Constant(3, 3, 0);
				}
				for (int k = 0; k < 2; k++) {
					for (int n = 0; n < N; n++) {
						t_0[k] = gamma_cal[n](k); T_0[k] += t_0[k];
						t_1[k] = t_0[k] * point[n]; T_1[k] += t_1[k];
						t_2[k] = t_1[k] * point[n].transpose(); T_2[k] += t_2[k];
					}
				}
				alpha_1 = 0.5;
				miu_1   = T_1[0] / T_0[0];
				sigma_1 = T_2[0] / T_0[0] - miu_1 * miu_1.transpose();

				alpha_2 = 0.5;
				miu_2   = T_1[1] / T_0[1];
				sigma_2 = T_2[1] / T_0[1] - miu_2 * miu_2.transpose();
				
			}

			double count_1 = 1.0;
			double count_2 = 1.0;
			for (int n = 0; n < N; n++) {
				if (gamma_cal[n](0) > 0.8) {
					gamma_cal[n](0) = 1;
					gamma_cal[n](1) = 0;
					count_1 += 1.0;
				}
				if (gamma_cal[n](1) > 0.8) {
					gamma_cal[n](1) = 1;
					gamma_cal[n](0) = 0;
					count_2 += 1.0;
				}
			}

			for (int i = 0; i < N; i++) {
				weight_1[i] = gamma_cal[i](0);
				weight_2[i] = gamma_cal[i](1);
			}

			Eigen::Vector3d temp_rate;
			temp_rate = cover_rate_cal(point,weight_1,weight_2,miu_1,miu_2,sigma_1,sigma_2,n_sigma,grid_size);

			parameter.alpha_k_1 = alpha_1;
			parameter.alpha_k_2 = alpha_2;
			// parameter.BIC = -weight_L_f * L_k1 + 2 *weight_node* log(N) - weight_cover*temp_rate(0);
			// parameter.BIC_1 = -weight_L_f * L_1 + 1 *weight_node* log(count_1)- weight_cover*temp_rate(1);
			// parameter.BIC_2 = -weight_L_f * L_1 + 1 *weight_node* log(count_2) - weight_cover*temp_rate(2);
			//std::cout<<-weight_L_f*L_k1<<";  "<<2.0*weight_node/(double)N<<";   "<<weight_cover*temp_rate(0)<<std:endl;
			// if(isnan(L_k1)) L_k1 = 0;
			// if(isnan(L_1))  L_1 = 0;
			// if(isnan(L_2))  L_2 = 0;

			parameter.BIC = -weight_L_f*L_k1 + 2.0*weight_node/(double)N - weight_cover*temp_rate(0);
			parameter.BIC_1 = -weight_L_f*L_1 + 1.0*weight_node/(double)count_1- weight_cover*temp_rate(1);
			parameter.BIC_2 = -weight_L_f*L_2 + 1.0*weight_node/(double)count_2 - weight_cover*temp_rate(2);
			//parameter.divide = ;
			parameter.R_cover = temp_rate(0);
			parameter.R_cover_1 =temp_rate(1);
			parameter.R_cover_2 = temp_rate(2);
			parameter.miu_k_1 = miu_1;
			parameter.miu_k_2 = miu_2;
			parameter.sigma_1 = sigma_1;
			parameter.sigma_2 = sigma_2;
			parameter.weight_1 = weight_1;
			parameter.weight_2 = weight_2;

			//std::cout << "step = " << step;
			//std::cout << "BIC = " << -2 * L_k1 + 2 * log(N) << std::endl;
			//std::cout << "miu_1 = " << miu_1 << std::endl;
			//std::cout << "miu_2 = " << miu_2 << std::endl;
			//std::cout << "sigma_1 = " << sigma_1 << std::endl;
			//std::cout << "sigma_2 = " << sigma_2 << std::endl;
			//for (int i = 0; i < N; i++) {
			//	std::cout << "weight = " << weight_1[i]<<", "<< weight_2[i] << std::endl;
			//}

			//std::cout<<step<<std::endl;
			return parameter;
		};

		double GaussianP(Eigen::Vector3d x, Eigen::Vector3d x_u, Eigen::Matrix3d var) {
			double PI = 3.1415926;
			double tmp = 1.0 / pow(2 * PI, 3.0 / 2.0) / 
				pow(var.determinant(), 0.5) * exp(-0.5 * (x - x_u).transpose() * var.inverse() * (x - x_u));
			return tmp;
		}
};

}

double getMahalanobisDistance(Eigen::Vector3d point,Eigen::Vector3d means,Eigen::Matrix3d var)
	{
			// double PI = 3.1415926;
			// double tmp = 1.0 / pow(2 * PI, 3.0 / 2.0) / 
			// 	pow(var.determinant(), 0.5) * exp(-0.5 * (point - means).transpose() * var.inverse() * (point - means));
			// return tmp;

		double distance=(point-means).transpose()*var.inverse()*(point-means);
		if(distance == 0) distance == 9999;
		return distance;
	}

Eigen::Vector3d cover_rate_cal(std::vector<Eigen::Vector3d> point,std::vector<double> weight1,
																		std::vector<double> weight2,Eigen::Vector3d mean1,Eigen::Vector3d mean2,
																		Eigen::Matrix3d var1,Eigen::Matrix3d var2, double n_sigma,double grid_size){
		
	
	Eigen::EigenSolver<Eigen::Matrix3d> var1_(var1);
	Eigen::EigenSolver<Eigen::Matrix3d> var2_(var2);
	Eigen::Matrix3d value1= var1_.pseudoEigenvalueMatrix();
	Eigen::Matrix3d value2= var2_.pseudoEigenvalueMatrix();
	// double V1;	V1 = 4.0/3.0*3.1415926*sqrt(value1(0,0))*sqrt(value1(1,1))*sqrt(value1(2,2))*pow(2.0,3.0);
	// double V2;	V2 = 4.0/3.0*3.1415926*sqrt(value2(0,0))*sqrt(value2(1,1))*sqrt(value2(2,2))*pow(2.0,3.0);
	double min_Gaussian = grid_size;//min Gaussina size
	for (int i = 0;i<3;i++){
		value1(i,i) = sqrt(value1(i,i));
		value2(i,i) = sqrt(value2(i,i));
		if(value1(i,i)<min_Gaussian)value1(i,i)=min_Gaussian;
		if(value2(i,i)<min_Gaussian)value2(i,i)=min_Gaussian;
	}
	value1 = value1*n_sigma*1;
	value2 = value2*n_sigma*1;
	double V1;	V1 = 4.0/3.0*3.1415926*value1(0,0)*value1(1,1)*value1(2,2);
	double V2;	V2 = 4.0/3.0*3.1415926*value2(0,0)*value2(1,1)*value2(2,2);
	// V1 = GMM_V_cal(var1, n_sigma,grid_size);
	// V2 = GMM_V_cal(var2, n_sigma,grid_size);

	// std::cout<<std::endl;
	// std::cout<<var1<<std::endl<<var2<<std::endl;
	// std::cout<<std::endl;
	// std::cout<<"feature vaule_1:"<<value1(0,0)<<","<<value1(1,1)<<","<<value1(2,2)<<std::endl;
	// std::cout<<"feature vaule_2:"<<value2(0,0)<<","<<value2(1,1)<<","<<value2(2,2)<<std::endl;

	// std::cout<<"V1:"<<V1<<std::endl;
	// std::cout<<"V2:"<<V2<<std::endl;

	int N;	N = point.size();

	//std::cout<<"N_size:"<<N<<std::endl;

	double only_point_1 = 0.0; double only_point_2 = 0.0;
	double only_GMM_1 = V1; double only_GMM_2 = V2;
	for(int i = 0;i<N;i++){
		if(weight1[i]>weight2[i]){
			if(getMahalanobisDistance(point[i],mean1,var1)>pow(n_sigma+2*grid_size,2)){
				only_point_1 += pow(grid_size,3);
			}
			else{
				//only_GMM_1 -= pow(grid_size*1.3,3);
				only_GMM_1 -= pow(grid_size,3);
				if (only_GMM_1<0.0)  only_GMM_1=0.0;
				//only_GMM_1 += pow(grid_size,3);
			}
		}
		else{
			if(getMahalanobisDistance(point[i],mean2,var2)>pow(n_sigma+2*grid_size,2)){
				only_point_2 += pow(grid_size,3);
			}
			else{
				//only_GMM_2 -= pow(grid_size*1.3,3);
				only_GMM_1 -= pow(grid_size,3);
				if (only_GMM_2<0.0)  only_GMM_2=0.0;
				//only_GMM_2 += pow(grid_size,3);
			}
		}
	}
	Eigen::Vector3d temp_rate;
	temp_rate(0) = 2*(V1+V2)/(2*(V1+V2)+only_point_1+only_point_2+only_GMM_1+only_GMM_2);
	temp_rate(1) = 2*V1/(2*V1+only_point_1+only_GMM_1);
	temp_rate(2) = 2*V2/(2*V2+only_point_2+only_GMM_2);
	// 运用占有率计算填充比率
	// temp_rate(0) = (V1+V2)/((V1+V2)+only_point_1+only_point_2);
	// temp_rate(1) = V1/(V1+only_point_1);
	// temp_rate(2) = V2/(V2+only_point_2);
	//std::cout<<"V_1:"<<V1<<" ,only_point_1:"<<only_point_1<<" ,only_GMM_1:"<<only_GMM_1<<" ,rate_1:"<<temp_rate(1)<<std::endl;
	//std::cout<<"V_2:"<<V2<<" ,only_point_2:"<<only_point_2<<" ,only_GMM_2:"<<only_GMM_2<<" ,rate_2:"<<temp_rate(2)<<std::endl;
	//std::cout<<"cover_rate:"<<temp_rate.transpose()<<std::endl;
	// std::cout<<std::endl;
	return temp_rate;
	}

	double GMM_V_cal(Eigen::Matrix3d var, double n_sigma,double grid_size){//目前没有使用 
		
		std::vector<Eigen::Vector3d> point_sample;
		pcl::PointCloud<pcl::PointXYZ>::Ptr gvmcbMap_pc(new pcl::PointCloud<pcl::PointXYZ>());
	
		gvm::GMM gmm_sample;
		std::vector<double> pi_vector; pi_vector.push_back(1.0);
		Eigen::Vector3d mean_zero;mean_zero<<0.0;0.0;0.0;
		std::vector<Eigen::Vector3d> means_vector; means_vector.push_back(mean_zero);
		std::vector<Eigen::Matrix3d> var_vector; var_vector.push_back(var);
		gmm_sample.init(pi_vector,means_vector,var_vector,means_vector.size(),means_vector[0]);
		point_sample = gmm_sample.GMMRandomSamplePoints(500,n_sigma);//2为最佳


		for(int i = 0;i<point_sample.size();i++){
			pcl::PointXYZ vis_point;
			vis_point.x = point_sample[i](0);
			vis_point.y = point_sample[i](1);
			vis_point.z = point_sample[i](2);
			gvmcbMap_pc->points.push_back(vis_point);
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(gvmcbMap_pc);
		static float res = grid_size;
		sor.setLeafSize(res, res, res);
		//sor.setMinimumPointsNumberPerVoxel(N_filter);
		sor.filter(*cloud_2);

		double GMM_V;
		GMM_V = double(cloud_2->points.size())*pow(grid_size,3.0);
		return GMM_V;
	}


#endif