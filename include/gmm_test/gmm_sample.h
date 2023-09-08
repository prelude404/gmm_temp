#ifndef GMM__H
#define GMM__H


#include <iostream>
#include <Eigen/Core>
#include <random>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

namespace gvm
{


struct normal_random_variable
{
    normal_random_variable(Eigen::MatrixXd const& covar)
        : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
        : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](double x) { return dist(gen); });
    }
};



class GMM
{
public:
	GMM(){}
	~GMM(){}
	GMM(std::vector<double> pi_vector,std::vector<Eigen::Vector3d> means, std::vector<Eigen::Matrix3d> var,int n_component,
		Eigen::Vector3d sensor_location):
		pi_vector_(pi_vector),
		means_(means),
		var_(var),
		n_component_(n_component),
		sensor_location_(sensor_location)
	{
		for(int i=0;i<n_component_;i++)
		{
			normal_random_variable random_variable(means[i].cast<double>(),var[i].cast<double>());
			randomGenerator.push_back(random_variable);
		}
	}

	std::vector<double> pi_vector_;
	std::vector<Eigen::Vector3d> means_;
	std::vector<Eigen::Matrix3d> var_;
	int n_component_;
	Eigen::Vector3d sensor_location_;
	int agent_id{0};
	int color_id{37};

	std::vector<normal_random_variable> randomGenerator;

public:

	inline double getMahalanobisDistance(Eigen::Vector3d point,Eigen::Vector3d means,Eigen::Matrix3d var)
	{
		double distance=(point-means).transpose()*var.inverse()*(point-means);
		return distance;
	}

	std::vector<Eigen::Vector3d> GMMRandomSamplePoints(int n_SamplePoints,double n_sigma)
	{
		std::vector<Eigen::Vector3d> samplePoints;
		for(int i=0;i<n_component_;i++)
		{
			int clusterPoint_num=n_SamplePoints*pi_vector_[i];
			for(int j=0;j<clusterPoint_num;j++)
			{
				Eigen::Vector3d tmp_point=randomGenerator[i]();

				if(getMahalanobisDistance(tmp_point,means_[i],var_[i])>pow(n_sigma,2))
					continue;
				samplePoints.push_back(tmp_point);
			}

		}

		return samplePoints;
	}

	void init(std::vector<double> pi_vector,std::vector<Eigen::Vector3d> means, std::vector<Eigen::Matrix3d> var,int n_component,
		Eigen::Vector3d sensor_location)
	{
		pi_vector_=pi_vector;
		means_=means;
		var_=var;
		n_component_=n_component;
		sensor_location_=sensor_location;
		randomGenerator.clear();
		for(int i=0;i<n_component_;i++)
		{
			normal_random_variable random_variable(means[i].cast<double>(),var[i].cast<double>());
			randomGenerator.push_back(random_variable);
		}
	}

	void setAgentId(int agentSetId, int colorSetId)
	{
		agent_id=agentSetId;
		color_id=colorSetId;
	}

	bool pointUnderFOV()
	{
		return true;

	}

	double getGaussianProbability(Eigen::VectorXd x,Eigen::VectorXd u,Eigen::MatrixXd var)
	{
		double PI=3.1415926;
		double tmp=1.0/pow(2*PI,1.5)/pow(var.determinant(),0.5)*exp(-0.5*(x-u).transpose()*var.inverse()*(x-u));
		// ROS_INFO_STREAM("GAUSSIAN u:"<<u<<" var: "<<var<<"var determinant: "<<var.determinant());
		// ROS_INFO("GAUSSIAN: %f",tmp);
		return tmp;
	}


};


}

#endif
