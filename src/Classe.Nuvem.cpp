#include "Classe.Nuvem.h"

TNuvem::TNuvem() : PontosRostoKinect(new NuvemXYZ())
{
	FaceKinect.reset(new NuvemRGB());
	FaceKinect->is_dense = true;

	CacheKinect.reset(new NuvemRGB());
	CacheKinect->is_dense = true;

	PontosRostoKinect.reset(new NuvemXYZ());
	PontosRostoKinect->is_dense = true;

	DistanciaOlhos = 999;
};

pcl::PointXYZRGB TNuvem::RetornaPontoKinectfrom2D(int x, int y)
{
	int aux = 0;

	//reduzir o x do width na chamada, ex: width - x
	pcl::PointXYZRGB p;

	p.x = 0;
	p.y = 0;
	p.z = 0;

	while (p.x == 0 && p.y == 0 && p.z == 0)
	{
		p.x = CacheKinect->points[y*width + x + aux].x;
		p.y = CacheKinect->points[y*width + x + aux].y;
		p.z = CacheKinect->points[y*width + x + aux].z;
		p.r = CacheKinect->points[y*width + x + aux].r;
		p.g = CacheKinect->points[y*width + x + aux].g;
		p.b = CacheKinect->points[y*width + x + aux].b;

		aux -= 2;
	}

	return p;
}

//Carrega a Nuvem da Face
bool TNuvem::CarregaFace(std::vector<pcl::PointXYZ> PontosRosto, bool limpa = false)
{
	bool retorno = true;

	if (PontosRosto.size() > 56)
	{
		//acima do 17 utilizar o shift
		int shiftPontos = 2;

		//Filtro Z
		//Nariz - 30
		int x = width - PontosRosto[30 + shiftPontos].x;
		int y = PontosRosto[30 + shiftPontos].y;
		pcl::PointXYZRGB pontoKinectNariz = RetornaPontoKinectfrom2D(x, y);

		//pegar da nuvem
		double FiltroZ = pontoKinectNariz.z;
		FiltroZ += 1.5 * DistanciaOlhos;

		//[16] - Ponto Limite Direito
		double xD = PontosRosto[16].x;
		//[0] - Ponto Limite Esquerdo
		double xE = PontosRosto[0].x;
		//[8] - Ponto Limite Baixo
		double yB = PontosRosto[8].y;
		//[17] - Ponto Limite Cima
		double yC = PontosRosto[17].y;

		int n = PontosRosto.size();
		for (int i = 0; i < n; i++)
		{
			if (PontosRosto[i].x > xD)
				xD = PontosRosto[i].x;
			if (PontosRosto[i].x < xE)
				xE = PontosRosto[i].x;

			if (PontosRosto[i].y > yB)
				yB = PontosRosto[i].y;
			if (PontosRosto[i].y < yC)
				yC = PontosRosto[i].y;
		}

		double inicio_faceX = width - xD;
		double fim_faceX = width - xE;

		double inicio_faceY = yC;
		double fim_faceY = yB;

		FaceKinect.reset(new NuvemRGB());
		FaceKinect->is_dense = true;
		//limite superior de pontos para nгo estourar no momento da aquisiзгo dos pontos da face, sendo redimensionado posteriormente
		FaceKinect->points.resize(CacheKinect->points.size());

		typedef boost::geometry::model::d2::point_xy<double> point_type;
		typedef boost::geometry::model::polygon<point_type> polygon_type;

		//Monta Poligono da Face
		polygon_type poligonoFace;
		for (int i = 0; i < 19; i++)
		{
			point_type p(PontosRosto[i].x, PontosRosto[i].y);
			poligonoFace.outer().push_back(p);
		}
		point_type pf(PontosRosto[0].x, PontosRosto[0].y);
		poligonoFace.outer().push_back(pf);

		//Monta Poligono Olho Direito
		polygon_type poligonoOlhoD;
		for (int i = 38; i < 44; i++)
		{
			try
			{
				point_type p(PontosRosto[i].x, PontosRosto[i].y);
				poligonoOlhoD.outer().push_back(p);
			}
			catch (std::exception&)
			{
				//do nothing
			}
		}
		point_type pd(PontosRosto[38].x, PontosRosto[38].y);
		poligonoOlhoD.outer().push_back(pd);

		//Monta Poligono Olho Esquerdo
		polygon_type poligonoOlhoE;
		for (int i = 44; i < 50; i++)
		{
			try
			{
				point_type p(PontosRosto[i].x, PontosRosto[i].y);
				poligonoOlhoE.outer().push_back(p);
			}
			catch (std::exception&)
			{
				//do nothing
			}
		}
		point_type pe(PontosRosto[44].x, PontosRosto[44].y);
		poligonoOlhoE.outer().push_back(pe);

		//Monta Poligono Boca
		double zFarBoca = 0;

		polygon_type poligonoBoca;
		int pontosPoligono[] = { 50, 6 , 8 , 10, 56, 50 };

		for (int i = 0; i < (sizeof(pontosPoligono) / sizeof(int)); ++i)
		{
			try
			{
				int index = pontosPoligono[i];

				point_type p(PontosRosto[index].x, PontosRosto[index].y);
				poligonoBoca.outer().push_back(p);
			}
			catch (std::exception&)
			{
				//do nothing
			}
		}

		pcl::PointXYZRGB pontoKinect = RetornaPontoKinectfrom2D(width - PontosRosto[5].x, PontosRosto[5].y);
		zFarBoca = pontoKinect.z;

		pontoKinect = RetornaPontoKinectfrom2D(width - PontosRosto[11].x, PontosRosto[11].y);
		if (pontoKinect.z > zFarBoca)
			zFarBoca = pontoKinect.z;

		int i = 0;
		bool ponto = false;
		for (int y = inicio_faceY; y < fim_faceY; y++)
		{
			for (int x = inicio_faceX; x < fim_faceX; x++)
			{
				ponto = false;

				pcl::PointXYZRGB pontoKinect = RetornaPontoKinectfrom2D(x, y);
				if (pontoKinect.x != 0 || pontoKinect.y != 0 || pontoKinect.z != 0)
				{
					point_type p(width - x, y);
					if (pontoKinect.z <= FiltroZ && boost::geometry::within(p, poligonoFace))
					{
						if (limpa)
						{
							if (boost::geometry::within(p, poligonoBoca))
								ponto = (pontoKinect.z < zFarBoca);
							else if (!boost::geometry::within(p, poligonoOlhoD) && !boost::geometry::within(p, poligonoOlhoE))
								ponto = true;
						}
						else
							ponto = true;
					}
				}

				if (ponto)
				{
					FaceKinect->points[i].x = pontoKinect.x;
					FaceKinect->points[i].y = pontoKinect.y;
					FaceKinect->points[i].z = pontoKinect.z;
					FaceKinect->points[i].r = pontoKinect.r;
					FaceKinect->points[i].g = pontoKinect.g;
					FaceKinect->points[i].b = pontoKinect.b;
					FaceKinect->points[i].a = pontoKinect.a;

					i++;
				}
			}
		}

		//#Landmark
		//shift points
		int landmarkIndices[] = {
			36 + shiftPontos,
			39 + shiftPontos,
			42 + shiftPontos,
			45 + shiftPontos,
			30 + shiftPontos,
			31 + shiftPontos,
			35 + shiftPontos,
			33 + shiftPontos,
			48 + shiftPontos,
			54 + shiftPontos
		};// ,
			//64 + shiftPontos,
			//61 + shiftPontos,
			//8 };
		for (int i = 0; i < (sizeof(landmarkIndices) / sizeof(int)); ++i)
		{
			int x = width - PontosRosto[landmarkIndices[i]].x;
			int y = PontosRosto[landmarkIndices[i]].y;

			pcl::PointXYZRGB pontoKinect = RetornaPontoKinectfrom2D(x, y);
			if (pontoKinect.x != 0 || pontoKinect.y != 0 || pontoKinect.z != 0)
			{
				FaceKinect->points[i].x = pontoKinect.x;
				FaceKinect->points[i].y = pontoKinect.y;
				FaceKinect->points[i].z = pontoKinect.z;
				FaceKinect->points[i].r = pontoKinect.r;
				FaceKinect->points[i].g = pontoKinect.g;
				FaceKinect->points[i].b = pontoKinect.b;
				FaceKinect->points[i].a = pontoKinect.a;

				i++;
			}
			else
				retorno = false;
		}

		//Specifies the height of the point cloud dataset in the number of points. height has two meanings:
		//it can specify the height (total number of rows) of an organized point cloud dataset;
		//it is set to 1 for unorganized datasets (thus used to check whether a dataset is organized or not).
		FaceKinect->height = 1;
		FaceKinect->width = i;

		//Redimensiona a nuvem somente com os pontos uteis
		FaceKinect->points.resize(i);
		//Carrega Nuvem Pontos Marcados do Rosto
		{
			int n = PontosRosto.size();

			PontosRostoKinect.reset(new NuvemXYZ());
			PontosRostoKinect->is_dense = true;
			PontosRostoKinect->height = 1;
			PontosRostoKinect->width = 3;
			PontosRostoKinect->points.resize(3);

			//Nuvem com os pontos capturados
			//Nariz - 30
			int x = width - PontosRosto[30 + shiftPontos].x;
			int y = PontosRosto[30 + shiftPontos].y;
			pcl::PointXYZRGB pontoKinectNariz = RetornaPontoKinectfrom2D(x, y);

			PontosRostoKinect->points[0].x = pontoKinectNariz.x;
			PontosRostoKinect->points[0].y = pontoKinectNariz.y;
			PontosRostoKinect->points[0].z = pontoKinectNariz.z;

			//OlhoD - 42 - x // 43 - y
			x = width - (PontosRosto[42 + shiftPontos].x + PontosRosto[45 + shiftPontos].x) / 2;
			y = (PontosRosto[43 + shiftPontos].y + PontosRosto[47 + shiftPontos].y) / 2;
			pcl::PointXYZRGB pontoKinectOlhoEsquerdo = RetornaPontoKinectfrom2D(x, y);

			PontosRostoKinect->points[1].x = pontoKinectOlhoEsquerdo.x;
			PontosRostoKinect->points[1].y = pontoKinectOlhoEsquerdo.y;
			PontosRostoKinect->points[1].z = pontoKinectOlhoEsquerdo.z;

			//OlhoE - 36 - x // 37 - y
			x = width - (PontosRosto[36 + shiftPontos].x + PontosRosto[39 + shiftPontos].x) / 2;
			y = (PontosRosto[37 + shiftPontos].y + PontosRosto[41 + shiftPontos].y) / 2;
			pcl::PointXYZRGB pontoKinectOlhoDireito = RetornaPontoKinectfrom2D(x, y);

			PontosRostoKinect->points[2].x = pontoKinectOlhoDireito.x;
			PontosRostoKinect->points[2].y = pontoKinectOlhoDireito.y;
			PontosRostoKinect->points[2].z = pontoKinectOlhoDireito.z;
		}
	}
	else
		retorno = false;

	return retorno;
};