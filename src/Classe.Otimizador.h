#pragma once
#ifndef __OTIMIZADOR_INCLUDED__
#define  __OTIMIZADOR_INCLUDED__

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/ply_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <dlib/optimization.h>
#include <iostream>
#include <vector>

#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "Classe.BlendShape.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> NuvemRGB;
typedef pcl::PointCloud<pcl::PointXYZ> NuvemXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGBA> NuvemRGBA;

using namespace pcl::io;
using namespace pcl::search;

typedef dlib::matrix<double, 0, 1> column_vector;

class TOtimizador
{
public:
	TOtimizador();

	TBlendShape blendshape;
	NuvemRGB::Ptr FaceKinect;

	std::vector<NuvemXYZ::Ptr> VetorExpressaoBlendShape;
	
	void CalculaIndicesMascara();

	pcl::PolygonMesh BlendShapeModificado;
	pcl::PolygonMesh BlendShapeOriginal;

	double Compute(NuvemRGB::Ptr, NuvemRGB::Ptr);
	double Distancias(const column_vector&);
	column_vector Calcula(column_vector);

	std::vector<int> *IndicesMascara;
	Eigen::Matrix<float, 3, Eigen::Dynamic> NuvemSource;
};

#endif