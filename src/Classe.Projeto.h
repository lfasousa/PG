#pragma once
#ifndef __PROJETO_INCLUDED__
#define  __PROJETO_INCLUDED__

#include <sstream>

#include <sys/stat.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/regex.hpp>

#include <pcl/filters/conditional_removal.h>

#include <pcl/common/time.h>

#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/icp.h>

#include "opencv2/highgui/highgui.hpp"

#include "Classe.BlendShape.h"
#include "Classe.Nuvem.h"
#include "Classe.Tracker.h"
#include "Classe.Otimizador.h"
#include "Classe.ModeloFace.h"
#include "Classe.PCA.h"

#include "kinect_grabber.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> NuvemRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> NuvemRGBA;

class TProjeto
{
public:
	//Limite : Kinect Kinect Xbox 360
	static const int width = 640;
	static const int height = 480;

	TProjeto();
	TBlendShape blendShape;
	TNuvem nuvem;
	TTracker tracker;
	TOtimizador Otimizador;

	int nProcessos;
	mutable boost::mutex mutexProcessos;

	boost::shared_ptr<pcl::Grabber> grabber;

	bool recorder;
	int recorder_frame;
	std::string rec_path;

	void CarregaEntrada();
	std::vector<NuvemRGB::Ptr> playerCache;

	bool calibrado;
	void Calibrar(NuvemRGB::Ptr);

	void Renderizacao();
	void Renderiza(std::string, NuvemRGB::Ptr);

	void Expressao();

	mutable boost::mutex mutex;
	mutable boost::mutex playermutex;
	std::vector<NuvemRGB::Ptr> cacheKinect;
	std::vector<pcl::PolygonMesh> resultado;
	std::vector<pcl::PolygonMesh> resultadoOriginal;

	int shiftPontos;

	pcl::visualization::PCLVisualizer Visualizer;
	cv::Mat ImagemKinectCache;

	//Callbacks
	void CallBack_Keyboard(const pcl::visualization::KeyboardEvent&);
	void CallBack_Cloud(const NuvemRGB::ConstPtr&);

	void TProjeto::Calibrar_blendshapeResult();
	void TProjeto::Calibrar_faceResult(NuvemRGB::Ptr);

	void AjustaDistanciaOlhos();

	double escala;
	void AjustaEscala();

	Eigen::Matrix4f PreAlinhamentoSVD(bool);
	Eigen::Matrix4f ICP();

	void Run(int);

	//Aux
	bool TProjeto::FileExists(const std::string&);
	void MontaViewport(pcl::visualization::PCLVisualizer&, std::string, int, float, float, float);
	pcl::PointXYZ ProdutoVetorial(pcl::PointXYZ, pcl::PointXYZ);
};

#endif