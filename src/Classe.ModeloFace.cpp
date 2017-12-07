#include "Classe.ModeloFace.h"

TModeloFace::TModeloFace()
{
	this->cloud = NuvemRGB::Ptr(new NuvemRGB);
}

void TModeloFace::updateLandmarks(NuvemRGB::Ptr cloudLandmark, std::vector<cv::Point> landmarksTracker)
{
	landmarks.clear();
	landmarks.resize(landmarksTracker.size());
	for (int i = 0; i < landmarksTracker.size(); ++i)
	{
		int x = landmarksTracker[i].x;
		int y = landmarksTracker[i].y;

		pcl::PointXYZRGB cp = cloudLandmark->points[(y*width) + (width - x)];

		pcl::PointXYZ p;
		p.x = cp.x;
		p.y = cp.y;
		p.z = cp.z;

		landmarks[i] = p;
	}
}

TModeloFace::TModeloFace(std::string cloudFileName, std::string landmarksFileName)
{
	this->cloud = CarregaCloud(cloudFileName);
	this->faces = CarregaFace(cloudFileName);

	if (landmarksFileName != "")
	{
		//#Landmark

		landmarks.resize(11); //multi completo
		//landmarks.resize(6); //pca

		landmarks.clear();
		ifstream file(landmarksFileName.c_str());
		for (int i = 0; i < (sizeof(landmarks) / sizeof(pcl::PointXYZ)); ++i)
			file >> landmarks[i].x >> landmarks[i].y >> landmarks[i].z;
		file.close();
	}
}

TModeloFace::TModeloFace(NuvemRGB::Ptr cloud, std::vector<pcl::PointXYZ> landmarks)
{
	this->cloud = cloud;
	this->landmarks = landmarks;
};

NuvemRGB::Ptr TModeloFace::CarregaCloud(std::string FileName)
{
	NuvemRGB::Ptr cloud(new NuvemRGB);
	ifstream file(FileName.c_str());
	std::string offType;

	int nPoints = 0, nFaces = 0, nEdges = 0;

	// read number of points, faces and edges of the cloud
	std::getline(file, offType);
	file >> nPoints >> nFaces >> nEdges;

	// read points data
	for (int i = 0; i < nPoints; ++i)
	{
		pcl::PointXYZRGB p;
		int temp_r = 255, temp_g = 255, temp_b = 255, temp_a = 0;

		file >> p.x >> p.y >> p.z;

		if (offType == "COFF")
			file >> temp_r >> temp_g >> temp_b >> temp_a;

		p.r = temp_r; p.g = temp_g; p.b = temp_b; p.a = temp_a;

		cloud->push_back(p);
	}

	file.close();
	return cloud;
}

std::vector<pcl::Vertices> TModeloFace::CarregaFace(std::string FileName)
{
	ifstream file(FileName.c_str());
	std::string line;
	int nPoints = 0, nFaces = 0, nEdges = 0;

	std::getline(file, line);

	file >> nPoints >> nFaces >> nEdges;
	std::getline(file, line);

	for (int i = 0; i < nPoints; ++i)
		std::getline(file, line);

	std::vector<pcl::Vertices> faces;

	//remove a face costurada
	nFaces -= 39;

	for (int i = 0; i < nFaces; ++i)
	{
		int nVertices;
		file >> nVertices;

		pcl::Vertices face;

		for (int j = 0; j < nVertices; ++j)
		{
			int verticeIndex;
			file >> verticeIndex;
			face.vertices.push_back(verticeIndex);
		}

		faces.push_back(face);
	}

	file.close();
	return faces;
}

void TModeloFace::Salvar(std::string cloudFileName, std::string landmarksFileName)
{
	//Save cloud OFF
	std::ofstream file(cloudFileName.c_str());
	int numValidPoints = 0;

	// write header
	file << "COFF" << std::endl;

	// number of not NaN points
	for (int i = 0; i < cloud->size(); ++i)
	{
		pcl::PointXYZRGB p = cloud->at(i);
		if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z))
			numValidPoints++;
	}

	file << numValidPoints << " " << faces.size() << " " << 0 << std::endl;

	// write points
	for (int i = 0; i < cloud->size(); ++i)
	{
		pcl::PointXYZRGB p = cloud->at(i);
		if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z))
			file << float(p.x) << " " << float(p.y) << " " << float(p.z) << " " << int(p.r) << " " << int(p.g) << " " << int(p.b) << " " << int(p.a) << std::endl;
	}

	// write faces
	for (int i = 0; i < faces.size(); ++i)
	{
		std::vector<unsigned int> f = faces[i].vertices;
		file << f.size() << " ";
		for (int j = 0; j < f.size(); ++j)
			file << f[j] << " ";
		file << std::endl;
	}

	file.close();

	if (landmarksFileName != "")
	{
		//saveLandmarksToTXT
		std::ofstream landmarksFile(landmarksFileName.c_str());

		for (int i = 0; i < landmarks.size(); ++i)
		{
			landmarksFile << landmarks[i].x << std::endl;
			landmarksFile << landmarks[i].y << std::endl;
			landmarksFile << landmarks[i].z << std::endl;
		}

		landmarksFile.close();
	}
};
