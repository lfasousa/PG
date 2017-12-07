#pragma once
#ifndef __PCA_INCLUDED__
#define  __PCA_INCLUDED__

#include "PCA/FileLoader.h"
#include "PCA/FileWriter.h"
#include "PCA/Definitions.h"
#include "PCA/MathHelper.h"
#include "PCA/MultilinearModelHandler.h"
#include "PCA/KDTree3.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include <algorithm>

//Enable if landmark indices are loaded instead of landmark coordinates
//#define USE_LANDMARK_INDICES

class TPCA
{
public:
	const std::string sstrMMFileName = "../PCA/All_30_7.rmm";
	const std::string sstrTemplateMeshFileName = "../PCA/MeanFace.off";
	const std::string sstrTemplateLmksFileName = "../PCA/All_Lmks.txt";

	bool projectToMultilinearModel(
		const std::string& sstrTargetMeshFileName,
		const std::string& sstrTargetLmksFileName,
		const std::string& sstrFittingOutFileName);
	void outputHelp();
};

#endif