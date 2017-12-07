#include "Classe.Otimizador.h"

class FuncaoOtimizador
{
public:
	TOtimizador* Otimimizador;

	FuncaoOtimizador(TOtimizador* o)
	{
		Otimimizador = o;
	}

	double operator() (const column_vector& arg) const
	{
		return Otimimizador->Distancias(arg);
	}
};

TOtimizador::TOtimizador()
{
	try
	{
		BlendShapeModificado.polygons.clear();
		pcl::io::loadPLYFile("../BlendShapes/Emily_blendshapes/template_base.ply", BlendShapeModificado);

		BlendShapeOriginal.polygons.clear();
		pcl::io::loadPLYFile("../BlendShapes/Emily_blendshapes/template_base.ply", BlendShapeOriginal);

		NuvemXYZ::Ptr nuvemB0(new NuvemXYZ());
		pcl::PolygonMesh BlendShape0;
		pcl::io::loadPLYFile("../BlendShapes/Emily_blendshapes/template_base.ply", BlendShape0);
		fromPCLPointCloud2(BlendShape0.cloud, *nuvemB0);

		NuvemSource.resize(3, 3);

		NuvemSource(0, 0) = nuvemB0->points[207].x;
		NuvemSource(1, 0) = nuvemB0->points[207].y;
		NuvemSource(2, 0) = nuvemB0->points[207].z;

		NuvemSource(0, 1) = nuvemB0->points[3004].x;
		NuvemSource(1, 1) = nuvemB0->points[3004].y;
		NuvemSource(2, 1) = nuvemB0->points[3004].z;

		NuvemSource(0, 2) = nuvemB0->points[3029].x;
		NuvemSource(1, 2) = nuvemB0->points[3029].y;
		NuvemSource(2, 2) = nuvemB0->points[3029].z;

		CalculaIndicesMascara();
	}
	catch (std::exception& e)
	{
		cout << e.what() << endl;
	}
}

double TOtimizador::Compute(NuvemRGB::Ptr NuvemBlend, NuvemRGB::Ptr NuvemFace)
{
	if (!NuvemBlend->points.empty() && !NuvemFace->points.empty())
	{
		pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
		tree.setInputCloud(NuvemBlend);

		double distanciasAcumuladas = 0;
		int count = 1;

		for (size_t i = 0; i < NuvemFace->points.size(); ++i)
		{
			std::vector<int> indices(1);
			std::vector<float> sqr_distances(1);
			tree.nearestKSearch(NuvemFace->points[i], 1, indices, sqr_distances);

			double dist = abs(sqr_distances[0]);
			if (dist <= 1)
			{
				distanciasAcumuladas += dist;
				count++;
			}
		}

		return (distanciasAcumuladas / count);
	}
	return 999999999;
}

double TOtimizador::Distancias(const column_vector& m)
{
	NuvemXYZ::Ptr blendShapeModificadaCloud(new NuvemXYZ());
	fromPCLPointCloud2(blendshape.BlendShape.cloud, *blendShapeModificadaCloud);

	NuvemRGB::Ptr blendShapeModificadaCloud_msk(new NuvemRGB());
	blendShapeModificadaCloud_msk->resize(blendShapeModificadaCloud->size());

	for (size_t i = 0; i < blendShapeModificadaCloud->size(); ++i)
	{
		blendShapeModificadaCloud_msk->points[i].x = 0;
		blendShapeModificadaCloud_msk->points[i].y = 0;
		blendShapeModificadaCloud_msk->points[i].z = 0;
	}

	for (size_t i = 0; i < IndicesMascara->size(); ++i)
	{
		for (int k = 0; k < blendshape.NumeroExpressoes; k++)
		{
			blendShapeModificadaCloud->points[IndicesMascara->at(i)].x += (VetorExpressaoBlendShape[k]->points[IndicesMascara->at(i)].x * m(k));
			blendShapeModificadaCloud->points[IndicesMascara->at(i)].y += (VetorExpressaoBlendShape[k]->points[IndicesMascara->at(i)].y * m(k));
			blendShapeModificadaCloud->points[IndicesMascara->at(i)].z += (VetorExpressaoBlendShape[k]->points[IndicesMascara->at(i)].z * m(k));
		}

		blendShapeModificadaCloud_msk->points[IndicesMascara->at(i)].x = blendShapeModificadaCloud->points[IndicesMascara->at(i)].x;
		blendShapeModificadaCloud_msk->points[IndicesMascara->at(i)].y = blendShapeModificadaCloud->points[IndicesMascara->at(i)].y;
		blendShapeModificadaCloud_msk->points[IndicesMascara->at(i)].z = blendShapeModificadaCloud->points[IndicesMascara->at(i)].z;
	}

	if (blendshape.Debug)
	{
		for (int k = 0; k < blendshape.NumeroExpressoes; k++)
			pcl::console::print_value("%f ", m(k));
		pcl::console::print_info("\n");
		pcl::console::print_info("\n");

		pcl::io::savePLYFile("BlendShapes_Alterados/bmc.ply", blendshape.BlendShape);
		pcl::io::savePLYFile("BlendShapes_Alterados/bmo.ply", blendshape.BlendShapeOriginal);
		pcl::io::savePLYFile("BlendShapes_Alterados/face.ply", *FaceKinect);

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Otimizador Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->setCameraPosition(0.0145235, 0.0350728, 0.288242, 0.0097265, 0.005817, 0.728135, 0.015801, 0.997683, 0.0661802);
		viewer->setCameraFieldOfView(0.523599);
		viewer->setCameraClipDistances(0.32839, 0.58357);
		viewer->setPosition(0, 0);
		viewer->setSize(683, 384);

		pcl::PolygonMesh BlendShape;
		pcl::io::loadPLYFile("../BlendShapes/Emily_blendshapes/template_base.ply", BlendShape);
		toPCLPointCloud2(*blendShapeModificadaCloud, BlendShape.cloud);

		if (!viewer->updatePolygonMesh(BlendShape, "BlendShape_Debug"))
			viewer->addPolygonMesh(BlendShape, "BlendShape_Debug");

		if (!viewer->updatePointCloud(FaceKinect, "face_Debug"))
			viewer->addPointCloud(FaceKinect, "face_Debug");

		while (!viewer->wasStopped())
			viewer->spinOnce();

		for (int k = 0; k < blendshape.NumeroExpressoes; k++)
		{
			NuvemXYZ::Ptr BlendShape_Debug(new NuvemXYZ());
			fromPCLPointCloud2(blendshape.BlendShape.cloud, *BlendShape_Debug);

			for (size_t i = 0; i < IndicesMascara->size(); ++i)
			{
				BlendShape_Debug->points[IndicesMascara->at(i)].x += VetorExpressaoBlendShape[k]->points[IndicesMascara->at(i)].x;
				BlendShape_Debug->points[IndicesMascara->at(i)].y += VetorExpressaoBlendShape[k]->points[IndicesMascara->at(i)].y;
				BlendShape_Debug->points[IndicesMascara->at(i)].z += VetorExpressaoBlendShape[k]->points[IndicesMascara->at(i)].z;
			}

			//for (int i = 0; i < BlendShape_Debug->points.size(); ++i)
			//{
			//	BlendShape_Debug->points[i].x += VetorExpressaoBlendShape[k]->points[i].x;
			//	BlendShape_Debug->points[i].y += VetorExpressaoBlendShape[k]->points[i].y;
			//	BlendShape_Debug->points[i].z += VetorExpressaoBlendShape[k]->points[i].z;
			//}

			pcl::PolygonMesh BlendShape;
			pcl::io::loadPLYFile("../BlendShapes/Emily_blendshapes/template_base.ply", BlendShape);
			toPCLPointCloud2(*BlendShape_Debug, BlendShape.cloud);

			std::ostringstream ss;
			ss.clear();
			ss << k;

			pcl::io::savePLYFile("BlendShapes_Alterados/" + ss.str() + ".ply", BlendShape);
		}
	}

	//Face - FaceKinect
	//BlendShape Modificada - blendShapeModificadaCloud
	//Calcular distância entre as duas
	double distancia = TOtimizador::Compute(blendShapeModificadaCloud_msk, FaceKinect);

	//cout << distancia << endl;
	return distancia;
}

void TOtimizador::CalculaIndicesMascara()
{
	IndicesMascara = new std::vector<int>();
	IndicesMascara->clear();

	pcl::PolygonMesh BlendShapeMascara;
	pcl::io::loadPLYFile("../BlendShapes/otm_Mask.ply", BlendShapeMascara);
	NuvemRGB::Ptr nuvemMascara(new NuvemRGB());
	fromPCLPointCloud2(BlendShapeMascara.cloud, *nuvemMascara);

	int c = 0;
	for (size_t i = 0; i < nuvemMascara->points.size(); ++i)
		if (nuvemMascara->points[i].r == 170)
			IndicesMascara->push_back(i);
}

column_vector TOtimizador::Calcula(column_vector starting_point)
{
	try
	{
		NuvemXYZ::Ptr blendShapeCloud(new NuvemXYZ());
		fromPCLPointCloud2(blendshape.BlendShape.cloud, *blendShapeCloud);

		Eigen::Matrix<float, 3, Eigen::Dynamic> NuvemTarget(3, 3);

		NuvemTarget(0, 0) = blendShapeCloud->points[207].x;
		NuvemTarget(1, 0) = blendShapeCloud->points[207].y;
		NuvemTarget(2, 0) = blendShapeCloud->points[207].z;

		NuvemTarget(0, 1) = blendShapeCloud->points[3004].x;
		NuvemTarget(1, 1) = blendShapeCloud->points[3004].y;
		NuvemTarget(2, 1) = blendShapeCloud->points[3004].z;

		NuvemTarget(0, 2) = blendShapeCloud->points[3029].x;
		NuvemTarget(1, 2) = blendShapeCloud->points[3029].y;
		NuvemTarget(2, 2) = blendShapeCloud->points[3029].z;

		Eigen::Matrix4f MatrizTransformacao = Eigen::umeyama(NuvemSource, NuvemTarget, true);

		MatrizTransformacao(0, 3) = 0;
		MatrizTransformacao(1, 3) = 0;
		MatrizTransformacao(2, 3) = 0;

		VetorExpressaoBlendShape.clear();
		VetorExpressaoBlendShape.resize(blendshape.NumeroExpressoes);

		for (int k = 0; k < blendshape.NumeroExpressoes; k++)
		{
			VetorExpressaoBlendShape[k].reset(new NuvemXYZ());
			VetorExpressaoBlendShape[k]->is_dense = true;
			VetorExpressaoBlendShape[k]->resize(blendshape.VetorExpressaoBlendShape[k]->points.size());

			pcl::transformPointCloud(*blendshape.VetorExpressaoBlendShape[k], *VetorExpressaoBlendShape[k], MatrizTransformacao);
		}

		dlib::find_min_bobyqa
		(
			FuncaoOtimizador(this),
			starting_point,
			starting_point.size() * 2 + 1,
			dlib::uniform_matrix<double>(starting_point.size(), 1, 0.0),  // lower bound constraint
			dlib::uniform_matrix<double>(starting_point.size(), 1, 1.0),   // upper bound constraint
			0.01,    // initial trust region radius
			0.001,  // stopping trust region radius
			10000    // max number of objective function evaluations
		);

		//Modificado com a face e 
		//Avatar Original
		{
			NuvemXYZ::Ptr blendShapeCloud(new NuvemXYZ());
			fromPCLPointCloud2(blendshape.BlendShape.cloud, *blendShapeCloud);

			NuvemXYZ::Ptr blendShapeOriginalCloud(new NuvemXYZ());
			fromPCLPointCloud2(blendshape.BlendShapeOriginal.cloud, *blendShapeOriginalCloud);

			for (int i = 0; i < blendShapeCloud->points.size(); ++i)
			{
				float x = 0;
				float y = 0;
				float z = 0;

				for (int k = 0; k < blendshape.NumeroExpressoes; k++)
				{
					x += VetorExpressaoBlendShape[k]->points[i].x * starting_point(k);
					y += VetorExpressaoBlendShape[k]->points[i].y * starting_point(k);
					z += VetorExpressaoBlendShape[k]->points[i].z * starting_point(k);
				}

				blendShapeCloud->points[i].x += x;
				blendShapeCloud->points[i].y += y;
				blendShapeCloud->points[i].z += z;

				blendShapeOriginalCloud->points[i].x += x;
				blendShapeOriginalCloud->points[i].y += y;
				blendShapeOriginalCloud->points[i].z += z;
			}

			BlendShapeModificado.polygons.clear();
			pcl::io::loadPLYFile("../BlendShapes/Emily_blendshapes/template_base.ply", BlendShapeModificado);
			toPCLPointCloud2(*blendShapeCloud, BlendShapeModificado.cloud);

			BlendShapeOriginal.polygons.clear();
			pcl::io::loadPLYFile("../BlendShapes/Emily_blendshapes/template_base.ply", BlendShapeOriginal);
			toPCLPointCloud2(*blendShapeOriginalCloud, BlendShapeOriginal.cloud);

			if (blendshape.Debug)
			{
				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Otimizador Viewer"));
				viewer->setBackgroundColor(0, 0, 0);
				viewer->setCameraPosition(0.0145235, 0.0350728, 0.288242, 0.0097265, 0.005817, 0.728135, 0.015801, 0.997683, 0.0661802);
				viewer->setCameraFieldOfView(0.523599);
				viewer->setCameraClipDistances(0.32839, 0.58357);
				viewer->setPosition(0, 0);
				viewer->setSize(683, 384);

				if (!viewer->updatePolygonMesh(BlendShapeModificado, "BlendShape_Debug"))
					viewer->addPolygonMesh(BlendShapeModificado, "BlendShape_Debug");

				while (!viewer->wasStopped())
					viewer->spinOnce();
			}
		}

		for (int k = 0; k < blendshape.NumeroExpressoes; k++)
			pcl::console::print_value("%f ", starting_point(k));
		pcl::console::print_info("\n");

		return starting_point;
	}
	catch (std::exception& e)
	{
		cout << e.what() << endl;
		return starting_point;
	}
}