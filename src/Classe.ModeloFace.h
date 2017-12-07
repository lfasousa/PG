#pragma once
#ifndef __ModeloFace_INCLUDED__
#define  __ModeloFace_INCLUDED__

#include <ostream>
#include <sstream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv/cv.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> NuvemRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> NuvemRGBA;

class TModeloFace
{
public:
	//Limite : Kinect Kinect Xbox 360
	static const int width = 640;

	NuvemRGB::Ptr cloud;
	std::vector<pcl::Vertices> faces;
	std::vector<pcl::PointXYZ> landmarks;

	TModeloFace();
	TModeloFace(std::string, std::string);
	TModeloFace(NuvemRGB::Ptr, std::vector<pcl::PointXYZ>);

	void Salvar(std::string, std::string);

	NuvemRGB::Ptr CarregaCloud(std::string);
	std::vector<pcl::Vertices> CarregaFace(std::string);

	void updateLandmarks(NuvemRGB::Ptr, std::vector<cv::Point>);
};

#endif;