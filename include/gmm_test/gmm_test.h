#ifndef EM_fit_gy
#define EM_fit_gy

#include <iostream>
#include <Eigen/Dense>
#include <ctime>
#include <math.h>
#include <numeric>
#include <algorithm>

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

std::vector<Eigen::Vector3d> point_sample_(std::vector<Eigen::Vector3d> means,
							std::vector<Eigen::Matrix3d> var,std::vector<double> pi,int number,double n_sigma);
double getMahalanobisDistance(Eigen::Vector3d point,Eigen::Vector3d means,Eigen::Matrix3d var);

namespace gy {

	double weight_L_f = 16.0;
	double weight_node = 0.6;

	class point_banch {
	public:
		std::vector<Eigen::Vector3d> point;//点云
		std::vector<double> weight;//每个点的权重
		bool mark = 0;//该点云是否需要被再次分解
		double BIC_value = 999999;//该点云的BIC数值

		//GMM簇参数
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
		Eigen::Matrix3d sigma_1;
		Eigen::Matrix3d sigma_2;

	};

	
	class GMM_fit {
	public:
		std::vector<Eigen::Vector3d>  point;
		std::vector<Eigen::Vector3d>  miu;
		std::vector<Eigen::Matrix3d>  sigma;
		std::vector<double> alpha;
		int max_number = 100;
		int EM_begin_number = 150;

	public:
		void GMM_fit_layer() {
			//初始化点云二叉树
			std::vector<point_banch> point_tree;
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
                if(point_tree.size()>30)break;			
				for (int i = 0; i < point_tree.size(); i++) {
					if (point_tree[i].mark == 0) {//若该树杈等待被分解，则开始分解
						if (point_tree[i].point.size()<10) {
							point_tree[i].mark = 1;
							break;
						}
						return_parameter tmp_parameter;
						//先由Kmeans算法初始化分解
						tmp_parameter = K_means_cal(point_tree[i].point);
						//若点个数足够小，则由EM算法继续拟合
						if (point_tree[i].point.size() < EM_begin_number) {
							tmp_parameter = EM_cal(point_tree[i].point,tmp_parameter);
						}
						//若BIC满足条件，则不进行进一步分裂
						if (tmp_parameter.BIC > point_tree[i].BIC_value && point_tree[i].point.size() < max_number) {
							point_tree[i].mark = 1;
							break;
						}
						point_banch point_banch_tmp_0;
						point_banch point_banch_tmp_1;

						for (int j = 0; j < point_tree[i].point.size(); j++) {
							if (tmp_parameter.weight_1[j] > tmp_parameter.weight_2[j]) {
								point_banch_tmp_0.point.push_back(point_tree[i].point[j]);
								point_banch_tmp_0.weight.push_back(point_tree[i].weight[j] * tmp_parameter.weight_1[j]);
							}
							else {
								point_banch_tmp_1.point.push_back(point_tree[i].point[j]);
								point_banch_tmp_1.weight.push_back(point_tree[i].weight[j] * tmp_parameter.weight_2[j]);
							}
						}
						point_banch_tmp_0.BIC_value = tmp_parameter.BIC_1;
						point_banch_tmp_1.BIC_value = tmp_parameter.BIC_2;
						point_banch_tmp_0.alpha_k = tmp_parameter.alpha_k_1 * point_tree[i].alpha_k;
						point_banch_tmp_1.alpha_k = tmp_parameter.alpha_k_2 * point_tree[i].alpha_k;
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
			}
			
			for (int i = 0; i < point_tree.size(); i++) {
				if(point_tree[i].sigma_k.sum() == 0) break;
				if(point_tree[i].miu_k.sum() == 0) break;
				
				alpha.push_back((double)point_tree[i].point.size()/(double)point.size());
				miu.push_back(point_tree[i].miu_k);
				sigma.push_back(point_tree[i].sigma_k);
			}

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
					L_1 += gamma_cal[n](0);
				else
					L_2 += gamma_cal[n](1);
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
			tmp_parameter.alpha_k_1 = alpha_1;
			tmp_parameter.alpha_k_2 = alpha_2;
			tmp_parameter.BIC = -weight_L_f*L_k1 + 2*weight_node* log(N);
			tmp_parameter.BIC_1 = -weight_L_f*L_1 + 2*weight_node* log(count_1);
			tmp_parameter.BIC_2 = -weight_L_f*L_2 + 2*weight_node* log(count_2);
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

			double alpha_1 = tmp_parameter.alpha_k_1;
			double alpha_2 = tmp_parameter.alpha_k_2;
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
						L_1 += gamma_cal[n](0);
					else
						L_2 += gamma_cal[n](1);
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

			parameter.alpha_k_1 = alpha_1;
			parameter.alpha_k_2 = alpha_2;
			parameter.BIC = -weight_L_f * L_k1 + 2 *weight_node* log(N);
			parameter.BIC_1 = -weight_L_f * L_1 + 2 *weight_node* log(count_1);
			parameter.BIC_2 = -weight_L_f * L_1 + 2 *weight_node* log(count_2);
			//parameter.divide = ;
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


std::vector<Eigen::Vector3d> point_sample_(std::vector<Eigen::Vector3d> means,
							std::vector<Eigen::Matrix3d> var,std::vector<double> pi,int number,double n_sigma){
	std::vector<Eigen::Vector3d> point_sample;		
	srand((unsigned)time(NULL));
	for(int  i = 0;i<means.size();i++){
		int number_sample = (double)number*pi[i];
		if(means[i](2) <= 0.5) continue;
		//std::cout<<"means = "<<means[i].transpose()<<std::endl;
		for(int j = 0;j<number_sample ; j++){
			Eigen::Vector3d tmp;
			//Eigen::Vector3d tmp=randomGenerator[i]();
			//tmp =  Eigen::Vector3d::Random();
			tmp =  Eigen::MatrixXd::Random(3,1);	
			//tmp = var[i]*tmp.transpose();
			if(getMahalanobisDistance(tmp,means[i],var[i])>pow(n_sigma,2))
			 	continue;
			point_sample.push_back(tmp);
		}
	}
	return point_sample;
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


#endif