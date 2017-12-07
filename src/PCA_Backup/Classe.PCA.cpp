#include "Classe.PCA.h"

void TPCA::outputHelp()
{
	std::cout << "Mesh fitting using a multilinear model" << std::endl;
	std::cout << "Parameters: " << std::endl;
	std::cout << " multilinearModel.rmm - full path of the used statistical multilinear model" << std::endl;
	std::cout << " templateMesh.off - full path of the training data template mesh, needed for the mesh structur of the result" << std::endl;
	std::cout << " templateLmks.txt - full path of a text file including the landmark coordinates of the template mesh, the first 8 landmarks are used to compute a rigid alignment" << std::endl;
	std::cout << " targetMesh.off - full path of the fitting target mesh" << std::endl;
	std::cout << " targetLmks.txt - full path of a text file including the landmark coordinates of the target face mesh, the first 8 landmarks are used to compute the rigid alignment" << std::endl;
	std::cout << " outFitting.off - full path of the fitting result file" << std::endl;
}

bool TPCA::projectToMultilinearModel(
	const std::string& sstrTargetMeshFileName, 
	const std::string& sstrTargetLmksFileName, 
	const std::string& sstrFittingOutFileName)
{
	//Load trained statistical model.
	MultilinearModelHandler multilinearModelHandler;
	if (!multilinearModelHandler.importMultilinearModel(sstrMMFileName))
	{
		std::cout << "Unable to import multilinear model " << sstrMMFileName << std::endl;
		return false;
	}

	FileLoader loader;

	//Load template mesh that defines the mesh structure of the fitted output. 
	//The template mesh is defined within the local coordinate system of the model.
	DataContainer templateMesh;
	if (!loader.loadFile(sstrTemplateMeshFileName, templateMesh))
	{
		std::cout << "Unable to load template mesh " << sstrTemplateMeshFileName << std::endl;
		return false;
	}

	const std::vector<double>& templateMeshVertices = templateMesh.getVertexList();

	//Load landmarks defined on the template mesh.
	std::vector<double> templateLandmarks;
	std::vector<bool> templateLandmarksLoaded;
	if (!loader.loadLandmarks(sstrTemplateLmksFileName, templateLandmarks, templateLandmarksLoaded))
	{
		std::cout << "Unable to load template landmarks " << sstrTemplateLmksFileName << std::endl;
		return false;
	}

	KDTree3 templateMeshKDTree(templateMeshVertices);

	//Load target data of the registration.
	DataContainer targetMesh;
	if (!loader.loadFile(sstrTargetMeshFileName, targetMesh))
	{
		std::cout << "Unable to load target file " << sstrTargetMeshFileName << std::endl;
		return false;
	}

	const std::vector<double>& targetMeshVertices = targetMesh.getVertexList();

	//Load landmarks defined at the target data.
	std::vector<double> targetLandmarks;
	std::vector<bool> targetLandmarksLoaded;
	if (!loader.loadLandmarks(sstrTargetLmksFileName, targetLandmarks, targetLandmarksLoaded))
	{
		std::cout << "Unable to load target landmark file " << sstrTargetLmksFileName << std::endl;
		return false;
	}

	std::vector<double> alignedTargetMeshVertices = targetMeshVertices;

	//Compute rigid transformation of the target data into local coordinate system of the model / template mesh.
	double s(0.0);
	std::vector<double> R;
	std::vector<double> t;
	if (!MathHelper::alignData(templateMeshVertices, templateLandmarks, templateLandmarksLoaded, alignedTargetMeshVertices, targetLandmarks, targetLandmarksLoaded, s, R, t, true, true))
	{
		std::cout << "Unable to compute alignment" << std::endl;
		return false;
	}

	/*Output initial alignment*/

	/**/


	//Compute projection into multilinear model space.
	std::vector<double> projectionWeights;
	multilinearModelHandler.projectToMM(alignedTargetMeshVertices, PROJECTION_MAX_POINT_DISTANCE, projectionWeights);

	//Compute the reconstruction from the model space. This is the representation of the target data within the multilinear model space.
	std::vector<double> fittingVertices;
	multilinearModelHandler.reconstructForWeights(projectionWeights, fittingVertices);
	templateMesh.setVertexList(fittingVertices);

	//Transform the fitting result back into the local coordinate system of the target data.
	double invs;
	std::vector<double> invR;
	std::vector<double> invt;
	MathHelper::invertTransformation(s, R, "N", t, "+", invs, invR, invt);

	MathHelper::transformMesh(invs, invR, "N", invt, "+", templateMesh);

	//Output registration result.
	if (!FileWriter::saveFile(sstrFittingOutFileName, templateMesh))
	{
		std::cout << "Unable to save file " << sstrFittingOutFileName << std::endl;
		return false;
	}

	return true;
}