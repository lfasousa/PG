#pragma once
#ifndef __NUVEM_INCLUDED__
#define  __NUVEM_INCLUDED__

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "opencv2/highgui/highgui.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <pcl/filters/filter.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> NuvemRGB;
typedef pcl::PointCloud<pcl::PointXYZ> NuvemXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGBA> NuvemRGBA;

class TNuvem
{
public:
	//Limite : Kinect Kinect Xbox 360
	static const int width = 640;
	static const int height = 480;

	TNuvem();

	NuvemRGB::Ptr CacheKinect;
	NuvemRGB::Ptr FaceKinect;

	double DistanciaOlhos;

	NuvemXYZ::Ptr PontosRostoKinect;

	bool CarregaFace(std::vector<pcl::PointXYZ>, bool);
	pcl::PointXYZRGB RetornaPontoKinectfrom2D(int, int);
};

#endif