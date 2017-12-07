#pragma once
#ifndef __BlendShape_INCLUDED__
#define  __BlendShape_INCLUDED__

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> NuvemXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> NuvemRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> NuvemRGBA;

class TBlendShape
{
public:
	bool Debug;
	TBlendShape();
	//Limite : Kinect Kinect Xbox 360
	static const int width = 640;
	static const int height = 480;

	double DistanciaOlhos;

	pcl::PolygonMesh BlendShape;
	pcl::PolygonMesh BlendShapeOriginal;

	NuvemRGB::Ptr NuvemMascara;

	int NumeroExpressoes;
	std::vector<pcl::PolygonMesh> ExpressaoBlendShape;
	std::vector<NuvemXYZ::Ptr> VetorExpressaoBlendShape;

	NuvemXYZ::Ptr PontosRostoBlendShape;
	void Reseta();
	void CriaMascara();
	void CalculaPontosRosto();
	void AjustaEscala(double);
	void CalculaExpressoes();
	bool FileExists(const std::string&);
};

#endif;