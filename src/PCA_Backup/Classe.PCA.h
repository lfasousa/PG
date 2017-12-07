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

class TPCA
{
public:
	const std::string sstrMMFileName = "../PCA/1.NE00_Scaled.rmm";
	const std::string sstrTemplateMeshFileName = "../PCA/2.MeanFace_NE00_Scaled.off";
	const std::string sstrTemplateLmksFileName = "../PCA/3.ScaledAlignedLandmarks.txt";

	bool projectToMultilinearModel(
		const std::string& sstrTargetMeshFileName, 
		const std::string& sstrTargetLmksFileName, 
		const std::string& sstrFittingOutFileName);
	void outputHelp();
};

#endif