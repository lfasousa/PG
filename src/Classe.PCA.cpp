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

	std::vector<double> templateMeshVertices;
	multilinearModelHandler.getDataMeanVertices(templateMeshVertices);
	templateMesh.setVertexList(templateMeshVertices);

	std::vector<double> templateMeshNormals;
	MathHelper::calcVertexNormals(templateMesh, templateMeshNormals);

	//Load landmarks defined on the template mesh.
	std::vector<double> templateLandmarks;
	std::vector<bool> templateLandmarksLoaded;
	/*Disable if landmark indices are loaded instead of landmark coordinates ->*/
#ifndef USE_LANDMARK_INDICES
	if (!loader.loadLandmarks(sstrTemplateLmksFileName, templateLandmarks, templateLandmarksLoaded))
	{
		std::cout << "Unable to load template landmarks " << sstrTemplateLmksFileName << std::endl;
		return false;
	}
#endif
	/*<- Disable if landmark indices are loaded instead of landmark coordinates*/

	/*Enable if landmark indices are loaded instead of landmark coordinates ->*/
#ifdef USE_LANDMARK_INDICES
	std::vector<size_t> templateLandmarkIndices;
	if (!loader.loadIndexFile(sstrTemplateLmksFileName, templateLandmarkIndices))
	{
		std::cout << "Unable to load model landmark indices" << std::endl;
		return false;
	}

	const size_t numTemplateLandmarks = templateLandmarkIndices.size();

	for (size_t i = 0; i < numTemplateLandmarks; ++i)
	{
		const size_t currLmkIndex = templateLandmarkIndices[i];
		templateLandmarks.push_back(templateMeshVertices[3 * currLmkIndex]);
		templateLandmarks.push_back(templateMeshVertices[3 * currLmkIndex + 1]);
		templateLandmarks.push_back(templateMeshVertices[3 * currLmkIndex + 2]);

		templateLandmarksLoaded.push_back(true);
	}
#endif
	/*<- Enable if landmark indices are loaded instead of landmark coordinates*/

	//Load landmarks defined at the target data.
	std::vector<double> targetLandmarks;
	std::vector<bool> targetLandmarksLoaded;
	if (!loader.loadLandmarks(sstrTargetLmksFileName, targetLandmarks, targetLandmarksLoaded))
	{
		std::cout << "Unable to load target landmark file " << sstrTargetLmksFileName << std::endl;
		return false;
	}

	/*Exchange this if oriented point clouds should be used ->*/
	//Load target data of the registration.
	DataContainer targetMesh;
	if (!loader.loadFile(sstrTargetMeshFileName, targetMesh))
	{
		std::cout << "Unable to load target file " << sstrTargetMeshFileName << std::endl;
		return false;
	}

	const std::vector<double>& targetMeshVertices = targetMesh.getVertexList();

	std::vector<double> targetMeshNormals;
	MathHelper::calcVertexNormals(targetMesh, targetMeshNormals);
	/*<- Exchange this if oriented point clouds should be used*/

	std::vector<double> alignedTargetMeshVertices = targetMeshVertices;
	std::vector<double> alignedTargetMeshNormals = targetMeshNormals;

	//Compute rigid transformation of the target data into local coordinate system of the model / template mesh.
	double s(0.0);
	std::vector<double> R;
	std::vector<double> t;
	if (!MathHelper::alignData(templateMeshVertices, templateMeshNormals, templateLandmarks, templateLandmarksLoaded, alignedTargetMeshVertices, alignedTargetMeshNormals, targetLandmarks, targetLandmarksLoaded, s, R, t, true, true))
	{
		std::cout << "Unable to compute alignment" << std::endl;
		return false;
	}

	/*Disable if landmark indices are loaded instead of landmark coordinates ->*/
#ifndef USE_LANDMARK_INDICES
	//Compute the indices of the template mesh landmarks.
	std::vector<size_t> templateLandmarkIndices;
	std::vector<double> targetProjectionLandmarks;

	KDTree3 templateMeshKDTree(templateMeshVertices);

	const size_t maxNumLmks = std::min<size_t>(templateLandmarksLoaded.size(), targetLandmarksLoaded.size());
	for (size_t i = 0; i < maxNumLmks; ++i)
	{
		if (!templateLandmarksLoaded[i] || !targetLandmarksLoaded[i])
		{
			continue;
		}

		std::vector<double> currLandmark;
		currLandmark.push_back(templateLandmarks[3 * i]);
		currLandmark.push_back(templateLandmarks[3 * i + 1]);
		currLandmark.push_back(templateLandmarks[3 * i + 2]);

		int pointIndex(0);
		double pointDistance(0.0);
		templateMeshKDTree.getNearestPoint(currLandmark, pointIndex, pointDistance);

		templateLandmarkIndices.push_back(static_cast<size_t>(pointIndex));

		targetProjectionLandmarks.push_back(targetLandmarks[3 * i]);
		targetProjectionLandmarks.push_back(targetLandmarks[3 * i + 1]);
		targetProjectionLandmarks.push_back(targetLandmarks[3 * i + 2]);
	}
#endif
	/*<- Disable if landmark indices are loaded instead of landmark coordinates*/

	//Transform the target landmarks used for fitting into local coordinate system of the model.
	MathHelper::transformData(s, R, "N", t, "+", targetProjectionLandmarks);

	//Compute projection into multilinear model space.
	std::vector<double> projectionWeights;
	multilinearModelHandler.projectToMM(templateMesh, templateLandmarkIndices, alignedTargetMeshVertices, alignedTargetMeshNormals, targetProjectionLandmarks, projectionWeights);

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