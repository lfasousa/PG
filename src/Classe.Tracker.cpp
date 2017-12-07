#include "Classe.Tracker.h"

const char *TTracker::traFile = "../BlendShapes/face2.tracker";

TTracker::TTracker() : FaceTracker(traFile)
{

}

bool TTracker::RastreiaRosto(cv::Mat ImagemGray)
{
	try 
	{
		bool failed = true;
		std::vector<int> wSize;
		PontosRosto.clear();

		wSize = failed ? boost::assign::list_of(11)(9)(7) : boost::assign::list_of(7);

		if (FaceTracker.Track(ImagemGray, wSize, -1, 10, 3, 0.01, true) == 0)
		{
			int idx = FaceTracker._clm.GetViewIdx();
			failed = false;

			cv::Mat& visi = FaceTracker._clm._visi[idx];
			int n = FaceTracker._shape.rows / 2;

			//#Landmark

			 //landmarks			( olhos,        nariz,			boca,	queixo)
			int landmarkIndices[] = { 36,39,42,45,  30,31,35,33,	48, 54 };//, 8 };

			landmarks.clear();
			for (int i = 0; i < (sizeof(landmarkIndices) / sizeof(int)); ++i)
			{
				int index = landmarkIndices[i];
				int x = FaceTracker._shape.at<double>(index, 0);
				int y = FaceTracker._shape.at<double>(index + n, 0);
				landmarks.push_back(cv::Point(x, y));
			}

			PontosRosto.clear();
			for (int i = 0; i < n; i++)
			{
				if (visi.at<int>(i, 0) == 0) continue;
				pcl::PointXYZ p;
				p.x = FaceTracker._shape.at<double>(i, 0);
				p.y = FaceTracker._shape.at<double>(i + n, 0);
				PontosRosto.push_back(p);
			}

			//Pontos Adicionais
			double OlhoE_x = (PontosRosto[36].x + PontosRosto[39].x) / 2;
			double OlhoE_y = (PontosRosto[37].y + PontosRosto[41].y) / 2;
			double OlhoD_x = (PontosRosto[42].x + PontosRosto[45].x) / 2;
			double OlhoD_y = (PontosRosto[43].y + PontosRosto[47].y) / 2;
			double Distancia_Olhos = OlhoE_x - OlhoD_x;

			//inserindo ordenado apos limite da cabeca
			pcl::PointXYZ p;
			p.x = OlhoD_x;
			p.y = OlhoD_y + Distancia_Olhos;
			PontosRosto.insert(PontosRosto.begin() + 17, p);

			p.x = OlhoE_x;
			p.y = OlhoE_y + Distancia_Olhos;
			PontosRosto.insert(PontosRosto.begin() + 18, p);
		}
		else
		{
			FaceTracker.FrameReset();
			failed = true;
		}

		return !failed;
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return false;
	}
}
