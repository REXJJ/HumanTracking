/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Author: Rex Jomy Joseph, University of Southern California         //
//  Email: jomyjose@usc.edu                                            //
//                                                                     // 
/////////////////////////////////////////////////////////////////////////

#ifndef HUMAN_BODY_HPP
#define HUMAN_BODY_HPP

/*******************************************/
//ROS HEADERS
/********************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>


/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <chrono>
#include <unordered_map> 
#include <queue>
#include <fstream>
#include <thread>
#include <ctime>

#include <cv.h>
#include <highgui.h>

#include <Eigen/Dense>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

namespace HumanTracking
{
	vector<vector<float>> getPart(const visualization_msgs::MarkerArray::ConstPtr& msg,vector<int> ids)
	{
		vector<vector<float>> ans;
		for(auto x:ids)
		{
			if(x>31||x<0)
				return {};
			auto joint=msg->markers[x];
			ans.push_back(vector<float>({joint.pose.position.x,joint.pose.position.y,joint.pose.position.z}));
		}
		return ans;
	}

	vector<vector<float>> getHead(const visualization_msgs::MarkerArray::ConstPtr& msg)
	{
		return getPart(msg,vector<int>({26,27,28,29,30,31}));
	}

	vector<vector<float>> getLeftHand(const visualization_msgs::MarkerArray::ConstPtr& msg)
	{
		return getPart(msg,vector<int>({5,6,7,8,9,10}));
	}

	vector<vector<float>> getRightHand(const visualization_msgs::MarkerArray::ConstPtr& msg)
	{
		return getPart(msg,vector<int>({12,13,14,15,16,17}));
	}

	inline Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
	{
		//! putting data in [x, y, z, 1]' format
		Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
		Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
		data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
		data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
		Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
		Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
		transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
		return transformed_data_mat.transpose();
	}

	vector<vector<float>> applyTransformation(const visualization_msgs::MarkerArray::ConstPtr& msg,Eigen::Matrix4d& transformation)
	{
		vector<vector<float>> ans;
		for(int i=0;i<msg->markers.size();i++)
		{
			auto joint=msg->markers[i];
			Eigen::MatrixXd pt(1,3);
			pt(0,0)=joint.pose.position.x;
			pt(0,1)=joint.pose.position.y;
			pt(0,2)=joint.pose.position.z;
			Eigen::MatrixXd pts_trans=apply_transformation(pt,transformation);
			ans.push_back({pts_trans(0,0),pts_trans(0,1),pts_trans(0,2)});
		}
		return ans;
	}
};



#endif