
# 基本介绍
本功能包主要利用分层GMM算法拟合点云，包括了拟合算法以及点云重构算法。拟合算法主要在文件：
* gmm_test.h

点云重构算法主要在文件：
* gmm_sample.h

算法需要安装pcl库以及eigen库才能正常运行。若未安装，请自行寻找资料安装。

 # 算法安装

您需要先在主目录下构建工作空间，创建src目录

```
cd 
mkdir -p ~/gmm_test_ws/src
cd ~/gmm_test_ws/src
```

 在目录下已经共享了功能包`gmm_test`。您可以克隆至新的工作空间下进行测试。执行命令：

```
git clone https://gitee.com/hatgao/gmm_fit.git
```


在`catkin_make`编译后，可以通过下述命令运行功能包：

 ```
source ./devel/setup.bash && roslaunch gmm_test gmm_start_all.launch
 ```


# 功能包接口

## 输入
* 无人机位姿  话题名称：/uav0/mavros/local_position/pose (0号无人机)；消息类型：geometry_msgs::PoseStamped；
* 相机转角 话题名称：/iris_0/gimbal_yaw_angle (0号无人机)；消息类型：geometry_msgs::PoseStamped；
* 点云 话题名称：/iris_0/camera/depth/points (0号无人机)；消息类型：sensor_msgs::PointCloud2；

## 输出
* 本架无人机接收的点云所拟合的高斯混合分布参数  话题名称：/gmm_share (所有无人机通用)；消息类型：gmm_test::GMM；
* 接收到的高斯混合分布重构的点云(包括自己发出的) 话题名称：/GMM_fitting0/gmm_cloud_sample (0号无人机)；消息类型：sensor_msgs::PointCloud2；



# 算法介绍

算法的拟合过程在函数`cloudCallback`中。若您想了解算法的具体流程，请参阅下面的介绍。也可以根据介绍，调整点云的拟合过程。源文件中标注了各个步骤的范围与功能，如下所示。

```C++
    //////////////////////////////////////////////STEP1//////////////////////////////////////////////
    //////////////////////////////////////////////直通滤波//////////////////////////////////////////////
```

## Step1

对点云进行直通滤波，除去距离摄像机较远以及距离地面较近的点云。您可以调节函数

```C++
pass.setFilterLimits (const float& min_limit, const float& max_limit)
```

的输入量，从而调整滤波的范围。

## Step2

对点云进行体素滤波，在这部分，您可以调节体素滤波的精度`res`。但不建议把该值设置过小，这会导致拟合的时间过长。

## Step3

这部分将点云旋转，需要提供无人机的位置姿态以及相机转角；并且转换点云格式，为欧式聚类作准备。

## Step4

对点云进行欧式聚类，您可以修改如下参数：

```C++
	ec.setClusterTolerance(0.3);	//设置近邻搜索的半径
	ec.setMinClusterSize(1);		//设置最小聚类点数
	ec.setMaxClusterSize(9999);	//设置最大聚类点数
```

但不建议做大幅度改动，有可能会影响正常的拟合。

## Step5

这部分是该算法的核心，可以定义`class gy:: GMM_fit`，先将点云`point_cloud`作为成员变量输入，然后调用函数`GMM.GMM_fit_layer()`，进行拟合，最终可以读取成员变量`miu`，`sigma`，`pi_vector`作为拟合结果。整个过程如下所示：

```C++
		gy::GMM_fit GMM;
		GMM.point = point_cloud;
		GMM.GMM_fit_layer();
		for(int i = 0;i<GMM.miu.size();i++){
			means.push_back(GMM.miu[i]);
			var.push_back(GMM.sigma[i]);
			pi_vector.push_back(GMM.pi_vector[i]);
		}
```

若您觉得拟合效果不够好，也可以通过修改成员变量`max_number`与`EM_begin_number`来修改`一簇点云最大的点云数量`，以及`开始利用EM算法(否则只使用K-means)聚类的点云数量`。


## Step6 & Step7

拟合结果可视化与发送GMM信息。
