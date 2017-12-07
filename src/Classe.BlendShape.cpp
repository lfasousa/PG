#include "Classe.BlendShape.h"

TBlendShape::TBlendShape() : PontosRostoBlendShape(new NuvemXYZ()), NuvemMascara(new NuvemRGB())
{
	NumeroExpressoes = 0;

	Debug = false;

	Reseta();

	CalculaExpressoes();

	CalculaPontosRosto();
};


bool TBlendShape::FileExists(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

void TBlendShape::CalculaExpressoes()
{
	//0 a 78
	for (int i = 0; i <= 78; i++)
	{
		std::string file_path;
		pcl::PolygonMesh expressao;
		std::stringstream ss;
		ss << i;

		file_path = "../BlendShapes/Emily_blendshapes/Utilizados/template_000" + ss.str() + ".ply";
		if (FileExists(file_path))
		{
			pcl::io::loadPLYFile(file_path, expressao);
			ExpressaoBlendShape.push_back(expressao);
		}
		else
		{
			file_path = "../BlendShapes/Emily_blendshapes/Utilizados/template_00" + ss.str() + ".ply";
			if (FileExists(file_path))
			{
				pcl::io::loadPLYFile(file_path, expressao);
				ExpressaoBlendShape.push_back(expressao);
			}
		}
	}

	NumeroExpressoes = ExpressaoBlendShape.size();

	VetorExpressaoBlendShape.clear();
	VetorExpressaoBlendShape.resize(NumeroExpressoes);

	NuvemXYZ::Ptr blendShapeCloud(new NuvemXYZ());
	fromPCLPointCloud2(BlendShape.cloud, *blendShapeCloud);

	for (int k = 0; k < NumeroExpressoes; k++)
	{
		VetorExpressaoBlendShape[k].reset(new NuvemXYZ());
		VetorExpressaoBlendShape[k]->is_dense = true;
		VetorExpressaoBlendShape[k]->resize(blendShapeCloud->points.size());

		NuvemXYZ::Ptr blendShapeExpressionCloud(new NuvemXYZ());
		fromPCLPointCloud2(ExpressaoBlendShape[k].cloud, *blendShapeExpressionCloud);

		for (int i = 0; i < blendShapeCloud->points.size(); ++i)
		{
			VetorExpressaoBlendShape[k]->points[i].x = blendShapeExpressionCloud->points[i].x - blendShapeCloud->points[i].x;
			VetorExpressaoBlendShape[k]->points[i].y = blendShapeExpressionCloud->points[i].y - blendShapeCloud->points[i].y;
			VetorExpressaoBlendShape[k]->points[i].z = blendShapeExpressionCloud->points[i].z - blendShapeCloud->points[i].z;
		}
	}
}

void TBlendShape::CriaMascara()
{
	//Criar a mascara para ICP
	NuvemRGB::Ptr NuvemBlendShapeFacePCA(new NuvemRGB());
	fromPCLPointCloud2(BlendShape.cloud, *NuvemBlendShapeFacePCA);

	pcl::PolygonMesh BlendShapeMascara;
	pcl::io::loadPLYFile("../BlendShapes/newMask.ply", BlendShapeMascara);
	NuvemRGB::Ptr nuvemMascara(new NuvemRGB());
	fromPCLPointCloud2(BlendShapeMascara.cloud, *nuvemMascara);

	NuvemMascara.reset(new NuvemRGB());
	NuvemMascara->is_dense = true;
	NuvemMascara->resize(NuvemBlendShapeFacePCA->points.size());

	int c = 0;
	for (size_t i = 0; i < nuvemMascara->points.size(); ++i)
		if (nuvemMascara->points[i].r == 170)
			NuvemMascara->points[c++] = NuvemBlendShapeFacePCA->points[i];

	NuvemMascara->resize(c - 1);
}

void TBlendShape::AjustaEscala(double escala)
{
	Eigen::Affine3f MatrizEscala = Eigen::Affine3f::Identity();
	MatrizEscala.scale(escala);

	NuvemXYZ::Ptr NuvemBlendShapeEscalado(new NuvemXYZ());
	fromPCLPointCloud2(BlendShape.cloud, *NuvemBlendShapeEscalado);
	pcl::transformPointCloud(*NuvemBlendShapeEscalado, *NuvemBlendShapeEscalado, MatrizEscala);
	toPCLPointCloud2(*NuvemBlendShapeEscalado, BlendShape.cloud);

	NuvemBlendShapeEscalado->clear();
	fromPCLPointCloud2(BlendShapeOriginal.cloud, *NuvemBlendShapeEscalado);
	pcl::transformPointCloud(*NuvemBlendShapeEscalado, *NuvemBlendShapeEscalado, MatrizEscala);
	toPCLPointCloud2(*NuvemBlendShapeEscalado, BlendShapeOriginal.cloud);

	pcl::transformPointCloud(*NuvemMascara, *NuvemMascara, MatrizEscala);
}

void TBlendShape::CalculaPontosRosto()
{
	NuvemXYZ::Ptr NuvemBlendShape(new NuvemXYZ());
	pcl::fromPCLPointCloud2(BlendShape.cloud, *NuvemBlendShape);
	PontosRostoBlendShape.reset(new NuvemXYZ());
	PontosRostoBlendShape->is_dense = true;
	PontosRostoBlendShape->height = 1;
	PontosRostoBlendShape->width = 3;
	PontosRostoBlendShape->points.resize(3);

	int PosNariz = 207;
	PontosRostoBlendShape->points[0].x = NuvemBlendShape->points[PosNariz].x;
	PontosRostoBlendShape->points[0].y = NuvemBlendShape->points[PosNariz].y;
	PontosRostoBlendShape->points[0].z = NuvemBlendShape->points[PosNariz].z;

	//3004
	//3002
	PontosRostoBlendShape->points[1].x = (NuvemBlendShape->points[3004].x + NuvemBlendShape->points[3002].x) / 2;
	PontosRostoBlendShape->points[1].y = (NuvemBlendShape->points[3004].y + NuvemBlendShape->points[3002].y) / 2;
	PontosRostoBlendShape->points[1].z = (NuvemBlendShape->points[3004].z + NuvemBlendShape->points[3002].z) / 2;

	double OlhoD_x = PontosRostoBlendShape->points[1].x;
	double OlhoD_y = PontosRostoBlendShape->points[1].y;
	double OlhoD_z = PontosRostoBlendShape->points[1].z;

	//3029 
	//3028		
	PontosRostoBlendShape->points[2].x = (NuvemBlendShape->points[3029].x + NuvemBlendShape->points[3028].x) / 2;
	PontosRostoBlendShape->points[2].y = (NuvemBlendShape->points[3029].y + NuvemBlendShape->points[3028].y) / 2;
	PontosRostoBlendShape->points[2].z = (NuvemBlendShape->points[3029].z + NuvemBlendShape->points[3028].z) / 2;

	double OlhoE_x = PontosRostoBlendShape->points[2].x;
	double OlhoE_y = PontosRostoBlendShape->points[2].y;
	double OlhoE_z = PontosRostoBlendShape->points[2].z;

	DistanciaOlhos = std::sqrt(std::pow((OlhoD_x - OlhoE_x), 2) + std::pow((OlhoD_y - OlhoE_y), 2) + std::pow((OlhoD_z - OlhoE_z), 2));
}

void TBlendShape::Reseta()
{
	pcl::io::loadPLYFile("../BlendShapes/Emily_blendshapes/template_base.ply", BlendShape);
	pcl::io::loadPLYFile("../BlendShapes/Emily_blendshapes/template_base.ply", BlendShapeOriginal);
};