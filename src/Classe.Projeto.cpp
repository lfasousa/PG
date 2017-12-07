#include "Classe.Projeto.h"
#include <time.h>

//Auxiliares
bool TProjeto::FileExists(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

struct sort_functor
{
	bool operator ()(const boost::filesystem::path & a, const boost::filesystem::path & b)
	{
		boost::regex re("(\\d+)");

		boost::match_results<std::string::const_iterator> what1;
		boost::regex_search(a.string().cbegin(), a.string().cend(), what1, re, boost::match_default);
		std::stringstream sa(what1[0]);
		int r1;
		sa >> r1 ? r1 : 0;

		boost::match_results<std::string::const_iterator> what2;
		boost::regex_search(b.string().cbegin(), b.string().cend(), what2, re, boost::match_default);
		std::stringstream sb(what2[0]);
		int r2;
		sb >> r2 ? r2 : 0;

		return r1 < r2;
	}
};

pcl::PointXYZ TProjeto::ProdutoVetorial(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
	pcl::PointXYZ n;
	n.x = (p1.y * p2.z) - (p1.z * p2.y);
	n.y = (p1.z * p2.x) - (p1.x * p2.z);
	n.z = (p1.x * p2.y) - (p1.y * p2.x);

	double lenN = sqrt(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2));
	n.x /= lenN;
	n.y /= lenN;
	n.z /= lenN;

	return n;
}

void TProjeto::MontaViewport(pcl::visualization::PCLVisualizer& pclVis, std::string viewportText, int viewport = 0, float bgColor_r = 0.0, float bgColor_g = 0.0, float bgColor_b = 0.0)
{
	// cor de fundo
	pclVis.setBackgroundColor(bgColor_r, bgColor_g, bgColor_b, viewport);

	// texto e id do viewport
	std::ostringstream viewportTextId;
	viewportTextId << "viewport" << viewport << " text";

	if (!pclVis.updateText(viewportText, 10, 10, 12, 1.0, 1.0, 1.0, viewportTextId.str()))
		pclVis.addText(viewportText, 10, 10, 12, 1.0, 1.0, 1.0, viewportTextId.str(), viewport);
}

//Controle
void TProjeto::CallBack_Keyboard(const pcl::visualization::KeyboardEvent& event)
{
	if (event.keyUp())
	{
		int c = event.getKeyCode();

		if (char(c) == 'K')
		{
			Calibrar(nuvem.FaceKinect);
		}
		else if (char(c) == 'F')
		{
			boost::thread(boost::bind(&TProjeto::Calibrar_faceResult, this, nuvem.FaceKinect));
		}
		else if (char(c) == 'T')
		{
			recorder = !recorder;
		}
		else if (char(c) == 'O')
		{
			Expressao();
		}
		else if (char(c) == 'D')
		{
			blendShape.Debug = !blendShape.Debug;
			cout << "Debug Otimizador: " << (blendShape.Debug ? "Ativado" : "Desativado") << endl;
		}
		else if (char(c) == 'M')
		{
			//ajusta escala
			AjustaEscala();

			pcl::io::loadPLYFile("Temp/BlendShape_Calibrado.ply", blendShape.BlendShape);
			pcl::io::loadPLYFile("Temp/BlendShape_Original.ply", blendShape.BlendShapeOriginal);

			//calcula mascara
			blendShape.CriaMascara();

			calibrado = true;
		}
		else if (char(c) == 'S')
		{
			int FileIndex = 0;

			std::stringstream ss;
			ss << FileIndex;
			while (FileExists("Cache/" + ss.str() + "-Cache.pcd"))
			{
				ss.str(std::string());
				ss.clear();

				FileIndex++;
				ss << FileIndex;
			}

			cout << "Nuvem Kinect salva " << ss.str() + "-Cache.pcd." << endl;
			pcl::io::savePCDFile("Cache/" + ss.str() + "-Cache.pcd", *nuvem.CacheKinect);
		}

		if (c >= 48 && c <= 57)
		{
			int indiceTeclado = c - '0';

			std::stringstream ss;
			ss.str(std::string());
			ss.clear();
			ss << indiceTeclado;

			if (FileExists("Cache/" + ss.str() + "-Cache.pcd"))
			{
				pcl::io::loadPCDFile("Cache/" + ss.str() + "-Cache.pcd", *nuvem.CacheKinect);

				for (int h = 0; h < ImagemKinectCache.rows; h++) {
					for (int w = 0; w < ImagemKinectCache.cols; w++) {
						pcl::PointXYZRGB point = nuvem.CacheKinect->at(w, h);
						Eigen::Vector3i rgb = point.getRGBVector3i();
						ImagemKinectCache.at<cv::Vec3b>(h, w)[0] = rgb[0];
						ImagemKinectCache.at<cv::Vec3b>(h, w)[1] = rgb[1];
						ImagemKinectCache.at<cv::Vec3b>(h, w)[2] = rgb[2];
					}
				}
			}
		}
	}
}

void TProjeto::Expressao()
{
	try
	{
		clock_t t0 = clock();

		NuvemRGB::Ptr faceKinectCloud(new NuvemRGB());
		copyPointCloud(*nuvem.FaceKinect, *faceKinectCloud);

		int numExpressoes = blendShape.NumeroExpressoes;
		column_vector starting_point(numExpressoes);
		for (int i = 0; i < numExpressoes; i++)
			starting_point(i) = 0;

		Otimizador.blendshape = blendShape;
		Otimizador.FaceKinect = faceKinectCloud;
		Otimizador.Calcula(starting_point);

		clock_t t1 = clock();
		printf("\nFrame : %.f\n", ((double)(t1 - t0) / (CLOCKS_PER_SEC / 1000)));

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Expressao"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->setCameraPosition(0.0145235, 0.0350728, 0.288242, 0.0097265, 0.005817, 0.728135, 0.015801, 0.997683, 0.0661802);
		viewer->setCameraFieldOfView(0.523599);
		viewer->setCameraClipDistances(0.32839, 0.58357);
		viewer->setPosition(0, 0);
		viewer->setSize(683, 384);

		int v1(1); viewer->createViewPort(0.00, 0.50, 0.50, 1.00, v1);	MontaViewport(*viewer, "Face", 1, 0.3, 0.3, 0.3);
		int v2(2); viewer->createViewPort(0.00, 0.00, 0.50, 0.50, v2);	MontaViewport(*viewer, "Modificado", 2, 0.3, 0.3, 0.3);
		int v3(3); viewer->createViewPort(0.50, 0.00, 1.00, 1.00, v3);	MontaViewport(*viewer, "Saida (Resultado)", 3, 0.3, 0.3, 0.3);

		viewer->addPointCloud(Otimizador.FaceKinect, "Face", 1);
		viewer->addPolygonMesh(Otimizador.BlendShapeModificado, "BlendShape Modificado", 2);
		viewer->addPolygonMesh(Otimizador.BlendShapeOriginal, "BlendShape Original", 3);

		while (!viewer->wasStopped())
			viewer->spinOnce();
	}
	catch (std::exception& e)
	{
		cout << e.what() << endl;
	}

}

void TProjeto::CallBack_Cloud(const NuvemRGB::ConstPtr& Kinect)
{
	copyPointCloud(*Kinect, *nuvem.CacheKinect);
}

//Execução
TProjeto::TProjeto() : Visualizer("Projeto de Graduação - Luiz Felipe de Abreu Sousa - 2017/2")
{
	escala = 0;

	nProcessos = 0;

	ImagemKinectCache.create(height, width, CV_8UC3);

	calibrado = false;

	recorder = false;
	recorder_frame = 0;

	//adicionei 2 pontos na face, calculados para a parte de cima da cabeça na parte de identificação da face
	shiftPontos = 2;

	try
	{
		grabber = boost::make_shared<KinectGrabber>();
		cout << "Kinect conectado!" << endl;
	}
	catch (std::exception&)
	{
		cout << "Kinect nao conectado! Utilizando a base local de dados." << endl;
		grabber = NULL;
	}

	if (grabber != 0)
	{
		boost::function<void(const NuvemRGB::ConstPtr&)> CallBackCloud_function = boost::bind(&TProjeto::CallBack_Cloud, this, _1);
		grabber->registerCallback(CallBackCloud_function);
		grabber->start();
	}
	else
		boost::thread(boost::bind(&TProjeto::CarregaEntrada, this));

	boost::function<void(const pcl::visualization::KeyboardEvent&)> CallBack_Key = boost::bind(&TProjeto::CallBack_Keyboard, this, _1);
	Visualizer.registerKeyboardCallback(CallBack_Key);
}

void TProjeto::CarregaEntrada()
{
	std::ostringstream ss;
	playerCache.clear();

	pcl::PCDReader r;
	NuvemRGB::Ptr aux;

	std::vector<boost::filesystem::path> v;
	std::copy(boost::filesystem::directory_iterator("Input/"), boost::filesystem::directory_iterator(), std::back_inserter(v));
	std::sort(v.begin(), v.end(), sort_functor());

	while (true)
	{
		for (std::vector<boost::filesystem::path>::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
		{
			while (playerCache.size() >= 30)
				boost::this_thread::sleep(boost::posix_time::millisec(2000));

			ss.str(std::string());
			ss.clear();
			ss << *it;

			aux.reset(new NuvemRGB());
			aux->is_dense = true;

			r.read(ss.str().substr(1, ss.str().length() - 2), *aux);

			playermutex.lock();
			if (std::find(playerCache.begin(), playerCache.end(), aux) == playerCache.end())
				playerCache.push_back(aux);
			playermutex.unlock();
		}
	}
}

Eigen::Matrix4f TProjeto::PreAlinhamentoSVD(bool escala = false)
{
	blendShape.CalculaPontosRosto();

	// Convert to Eigen format
	const int NumeroPontos = static_cast <int> ((*nuvem.PontosRostoKinect).size());

	Eigen::Matrix<float, 3, Eigen::Dynamic> NuvemSource(3, NumeroPontos);
	Eigen::Matrix<float, 3, Eigen::Dynamic> NuvemTarget(3, NumeroPontos);

	for (int i = 0; i < NumeroPontos; ++i)
	{
		NuvemSource(0, i) = blendShape.PontosRostoBlendShape->points[i].x;
		NuvemSource(1, i) = blendShape.PontosRostoBlendShape->points[i].y;
		NuvemSource(2, i) = blendShape.PontosRostoBlendShape->points[i].z;

		NuvemTarget(0, i) = nuvem.PontosRostoKinect->points[i].x;
		NuvemTarget(1, i) = nuvem.PontosRostoKinect->points[i].y;
		NuvemTarget(2, i) = nuvem.PontosRostoKinect->points[i].z;
	}

	Eigen::Matrix4f MatrizTransformacao = Eigen::umeyama(NuvemSource, NuvemTarget, escala);

	return MatrizTransformacao;
}

void TProjeto::Calibrar_faceResult(NuvemRGB::Ptr nuvemFace)
{
	try
	{
		TPCA PCA;

		//Face com landmarks e modelo calculado
		TModeloFace* faceCloud = new TModeloFace();
		faceCloud->cloud = nuvemFace;
		faceCloud->updateLandmarks(nuvem.CacheKinect, tracker.landmarks);
		faceCloud->Salvar("../PCA/4.faceCloud.off", "../PCA/5.faceLandmarks.txt");

		if (!PCA.projectToMultilinearModel("../PCA/4.faceCloud.off", "../PCA/5.faceLandmarks.txt", "../PCA/6.faceResult.off"))
			throw "Erro ao calibrar, tentando novamente!";

		pcl::io::savePLYFile("Temp/faceCloud.ply", *faceCloud->cloud);
	}
	catch (std::exception& e)
	{
		cout << e.what() << endl;
	}
}

void TProjeto::Calibrar_blendshapeResult()
{
	TPCA PCA;

	NuvemRGB::Ptr NuvemBlendShape(new NuvemRGB());
	fromPCLPointCloud2(blendShape.BlendShape.cloud, *NuvemBlendShape);

	TModeloFace* blendShapeCloud = new TModeloFace();
	blendShapeCloud->cloud = NuvemBlendShape;

	//#Landmark
	//						   olhos						//nariz			//boca		//queixo
	int landmarkIndices[] = { 3034,3025,	2996,2990,	910,2212,1467,13,   1770, 2945 };//, 337 };

	std::vector<pcl::PointXYZ> landmarks;
	for (int i = 0; i < (sizeof(landmarkIndices) / sizeof(int)); ++i)
	{
		float x = NuvemBlendShape->points[landmarkIndices[i]].x;
		float y = NuvemBlendShape->points[landmarkIndices[i]].y;
		float z = NuvemBlendShape->points[landmarkIndices[i]].z;
		landmarks.push_back(pcl::PointXYZ(x, y, z));
	}
	blendShapeCloud->landmarks = landmarks;

	blendShapeCloud->Salvar("../PCA/4.blendShapeCloud.off", "../PCA/5.blendshapeLandmarks.txt");

	if (!PCA.projectToMultilinearModel("../PCA/4.blendShapeCloud.off", "../PCA/5.blendshapeLandmarks.txt", "../PCA/6.blendshapeResult.off"))
		throw "Erro ao calibrar, tentando novamente!";

	pcl::io::savePLYFile("Temp/blendShapeCloud.ply", *blendShapeCloud->cloud);
}

void TProjeto::Calibrar(NuvemRGB::Ptr nuvemFace)
{
	try
	{
		cout << "Inicio Calibrar\n";
		clock_t t0 = clock();

		boost::thread* thfaceResult = new boost::thread(boost::bind(&TProjeto::Calibrar_faceResult, this, nuvemFace));
		boost::thread* thblendshapeResult = new boost::thread(boost::bind(&TProjeto::Calibrar_blendshapeResult, this));

		thfaceResult->join();
		TModeloFace* faceMesh = new TModeloFace("../PCA/6.faceResult.off", "");
		pcl::io::savePLYFile("Temp/faceMesh.ply", *faceMesh->cloud);
		delete thfaceResult;

		thblendshapeResult->join();
		TModeloFace* blendShapeResult = new TModeloFace("../PCA/6.blendshapeResult.off", "");
		pcl::io::savePLYFile("Temp/blendShapeResult.ply", *blendShapeResult->cloud);
		delete thblendshapeResult;

		//Captura de pontos BlendShape -> Modelo PCA calculado
		NuvemXYZ::Ptr NuvemBlendShapeFacePCA(new NuvemXYZ());
		fromPCLPointCloud2(blendShape.BlendShape.cloud, *NuvemBlendShapeFacePCA);
		toPCLPointCloud2(*NuvemBlendShapeFacePCA, blendShape.BlendShapeOriginal.cloud);

		std::vector<int> i_pontosFora;
		i_pontosFora.clear();
		std::vector<pcl::PointXYZ> pontosFora;
		pontosFora.clear();

		//para cada ponto na nuvem do "blendshape original" se busca o triangulo do "blendshape PCA" mais proxima a ele, 
		//e o modifica para aproxima-lo ao resultado da "face PCA"
		for (size_t i = 0; i < NuvemBlendShapeFacePCA->points.size(); ++i)
		{
			//ponto de busca
			pcl::PointXYZ p = NuvemBlendShapeFacePCA->points[i];
			//ponto de resultado
			pcl::PointXYZ pontoProximo;
			//face onde o ponto de resultado se encontra
			int closestFace = 0;

			double menorDistancia = 999999;
			for (size_t k = 0; k < blendShapeResult->faces.size(); ++k)
			{
				pcl::PointXYZ ponto;

				std::vector<unsigned int> f = blendShapeResult->faces[k].vertices;
				//triangulo da face (p1,p2,p3)
				pcl::PointXYZRGB p1 = blendShapeResult->cloud->at(f[0]);
				pcl::PointXYZRGB p2 = blendShapeResult->cloud->at(f[1]);
				pcl::PointXYZRGB p3 = blendShapeResult->cloud->at(f[2]);

				//1. normalizar ponto com triangulo p1*p2 x p1*p3
				pcl::PointXYZ p1p2;
				p1p2.x = p2.x - p1.x;
				p1p2.y = p2.y - p1.y;
				p1p2.z = p2.z - p1.z;

				pcl::PointXYZ p1p3;
				p1p3.x = p3.x - p1.x;
				p1p3.y = p3.y - p1.y;
				p1p3.z = p3.z - p1.z;

				pcl::PointXYZ n = ProdutoVetorial(p1p2, p1p3);

				//2. cos do angulo entre a normal e p1p0 
				pcl::PointXYZ p1p0;
				p1p0.x = p.x - p1.x;
				p1p0.y = p.y - p1.y;
				p1p0.z = p.z - p1.z;

				double cosAngle = (p1p0.x*n.x + p1p0.y*n.y + p1p0.z*n.z) / (sqrt(pow(p1p0.x, 2) + pow(p1p0.y, 2) + pow(p1p0.z, 2)) * sqrt(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2)));

				//3. calcular distancia entre ponto p e triangulo se cair dentro do triangulo (já normalizado)
				pcl::PointXYZ p0p1;
				p0p1.x = p1.x - p.x;
				p0p1.y = p1.y - p.y;
				p0p1.z = p1.z - p.z;
				double lenp0p1 = sqrt(pow(p0p1.x, 2) + pow(p0p1.y, 2) + pow(p0p1.z, 2));
				double lenp0 = lenp0p1 * cosAngle;

				//4. vetor entre p e p normalizado com o triangulo
				pcl::PointXYZ p0p_;
				p0p_.x = -lenp0 * n.x / sqrt(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2));
				p0p_.y = -lenp0 * n.y / sqrt(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2));
				p0p_.z = -lenp0 * n.z / sqrt(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2));

				//5. ponto p no plano do triangulo
				pcl::PointXYZ p_;
				p_.x = p.x + p0p_.x;
				p_.y = p.y + p0p_.y;
				p_.z = p.z + p0p_.z;

				double dist = std::abs(lenp0);
				ponto = p_;

				//baricentricas
				Eigen::Matrix<double, 3, 3> matrixPontos;
				matrixPontos(0, 0) = p1.x;	matrixPontos(0, 1) = p1.y;	matrixPontos(0, 2) = p1.z;
				matrixPontos(1, 0) = p2.x;	matrixPontos(1, 1) = p2.y;	matrixPontos(1, 2) = p2.z;
				matrixPontos(2, 0) = p3.x;	matrixPontos(2, 1) = p3.y;	matrixPontos(2, 2) = p3.z;

				Eigen::Matrix<double, 1, 3> matrixPonto;
				matrixPonto(0, 0) = p_.x;
				matrixPonto(0, 1) = p_.y;
				matrixPonto(0, 2) = p_.z;

				Eigen::Matrix<double, 1, 3> matrixAlphas = matrixPonto * matrixPontos.inverse();
				double a1 = matrixAlphas(0, 0);
				double a2 = matrixAlphas(0, 1);
				double a3 = matrixAlphas(0, 2);

				//fora do triangulo, procura o vertice mais proximo
				if (a1 > 1 || a2 > 1 || a3 > 1 || a1 < 0 || a2 < 0 || a3 < 0)
				{
					pcl::PointXYZ p2p1;
					p2p1.x = p1.x - p2.x;
					p2p1.y = p1.y - p2.y;
					p2p1.z = p1.z - p2.z;

					pcl::PointXYZ p3p1;
					p3p1.x = p1.x - p3.x;
					p3p1.y = p1.y - p3.y;
					p3p1.z = p1.z - p3.z;

					pcl::PointXYZ p3p2;
					p3p2.x = p2.x - p3.x;
					p3p2.y = p2.y - p3.y;
					p3p2.z = p2.z - p3.z;

					pcl::PointXYZ p2p3;
					p2p3.x = p3.x - p2.x;
					p2p3.y = p3.y - p2.y;
					p2p3.z = p3.z - p2.z;

					pcl::PointXYZ v1;
					v1.x = p2p1.x / sqrt(pow(p2p1.x, 2) + pow(p2p1.y, 2) + pow(p2p1.z, 2)) + p3p1.x / sqrt(pow(p3p1.x, 2) + pow(p3p1.y, 2) + pow(p3p1.z, 2));
					v1.y = p2p1.y / sqrt(pow(p2p1.x, 2) + pow(p2p1.y, 2) + pow(p2p1.z, 2)) + p3p1.y / sqrt(pow(p3p1.x, 2) + pow(p3p1.y, 2) + pow(p3p1.z, 2));
					v1.z = p2p1.z / sqrt(pow(p2p1.x, 2) + pow(p2p1.y, 2) + pow(p2p1.z, 2)) + p3p1.z / sqrt(pow(p3p1.x, 2) + pow(p3p1.y, 2) + pow(p3p1.z, 2));

					pcl::PointXYZ v2;
					v2.x = p3p2.x / sqrt(pow(p3p2.x, 2) + pow(p3p2.y, 2) + pow(p3p2.z, 2)) + p1p2.x / sqrt(pow(p1p2.x, 2) + pow(p1p2.y, 2) + pow(p1p2.z, 2));
					v2.y = p3p2.y / sqrt(pow(p3p2.x, 2) + pow(p3p2.y, 2) + pow(p3p2.z, 2)) + p1p2.y / sqrt(pow(p1p2.x, 2) + pow(p1p2.y, 2) + pow(p1p2.z, 2));
					v2.z = p3p2.z / sqrt(pow(p3p2.x, 2) + pow(p3p2.y, 2) + pow(p3p2.z, 2)) + p1p2.z / sqrt(pow(p1p2.x, 2) + pow(p1p2.y, 2) + pow(p1p2.z, 2));

					pcl::PointXYZ v3;
					v3.x = p1p3.x / sqrt(pow(p1p3.x, 2) + pow(p1p3.y, 2) + pow(p1p3.z, 2)) + p2p3.x / sqrt(pow(p2p3.x, 2) + pow(p2p3.y, 2) + pow(p2p3.z, 2));
					v3.y = p1p3.y / sqrt(pow(p1p3.x, 2) + pow(p1p3.y, 2) + pow(p1p3.z, 2)) + p2p3.y / sqrt(pow(p2p3.x, 2) + pow(p2p3.y, 2) + pow(p2p3.z, 2));
					v3.z = p1p3.z / sqrt(pow(p1p3.x, 2) + pow(p1p3.y, 2) + pow(p1p3.z, 2)) + p2p3.z / sqrt(pow(p2p3.x, 2) + pow(p2p3.y, 2) + pow(p2p3.z, 2));

					pcl::PointXYZ p1p_;
					p1p_.x = p_.x - p1.x;
					p1p_.y = p_.y - p1.y;
					p1p_.z = p_.z - p1.z;
					pcl::PointXYZ nf1 = ProdutoVetorial(v1, p1p_);
					double f1 = nf1.x * n.x + nf1.y * n.y + nf1.z * n.z;

					pcl::PointXYZ p2p_;
					p2p_.x = p_.x - p2.x;
					p2p_.y = p_.y - p2.y;
					p2p_.z = p_.z - p2.z;
					pcl::PointXYZ nf2 = ProdutoVetorial(v2, p2p_);
					double f2 = nf2.x * n.x + nf2.y * n.y + nf2.z * n.z;

					pcl::PointXYZ p3p_;
					p3p_.x = p_.x - p3.x;
					p3p_.y = p_.y - p3.y;
					p3p_.z = p_.z - p3.z;
					pcl::PointXYZ nf3 = ProdutoVetorial(v3, p3p_);
					double f3 = nf3.x * n.x + nf3.y * n.y + nf3.z * n.z;

					//posicoes
					bool antihorarioV1 = f1 > 0;
					bool antihorarioV2 = f2 > 0;
					bool antihorarioV3 = f3 > 0;

					pcl::PointXYZ p_p1;
					p_p1.x = p1.x - p_.x;
					p_p1.y = p1.y - p_.y;
					p_p1.z = p1.z - p_.z;

					pcl::PointXYZ p_p2;
					p_p2.x = p2.x - p_.x;
					p_p2.y = p2.y - p_.y;
					p_p2.z = p2.z - p_.z;

					pcl::PointXYZ p_p3;
					p_p3.x = p3.x - p_.x;
					p_p3.y = p3.y - p_.y;
					p_p3.z = p3.z - p_.z;

					// caso 1 : horario v2 e anti horario v1 - P1P2
					if (!antihorarioV2 && antihorarioV1)
					{
						pcl::PointXYZ r = ProdutoVetorial(ProdutoVetorial(p_p2, p_p1), p1p2);
						double cosAngle_r = (p_p1.x*r.x + p_p1.y*r.y + p_p1.z*r.z) / (sqrt(pow(p_p1.x, 2) + pow(p_p1.y, 2) + pow(p_p1.z, 2)) * sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2)));
						double length_r = sqrt(pow(p_p1.x, 2) + pow(p_p1.y, 2) + pow(p_p1.z, 2)) * cosAngle_r;

						pcl::PointXYZ p_p__;
						p_p__.x = length_r * r.x / sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2));
						p_p__.y = length_r * r.y / sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2));
						p_p__.z = length_r * r.z / sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2));

						pcl::PointXYZ p__;
						p__.x = p_.x + p_p__.x;
						p__.y = p_.y + p_p__.y;
						p__.z = p_.z + p_p__.z;

						ponto = p__;

						double t = sqrt(pow(p__.x - p1.x, 2) + pow(p__.y - p1.y, 2) + pow(p__.z - p1.z, 2)) / sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));

						if ((t >= 0) && (t <= 1))
							dist = sqrt(pow(length_r, 2) + pow(lenp0, 2));
						else if (t < 0)
							dist = sqrt(pow(p1.x - p.x, 2) + pow(p1.y - p.y, 2) + pow(p1.z - p.z, 2));
						else if (t > 1)
							dist = sqrt(pow(p2.x - p.x, 2) + pow(p2.y - p.y, 2) + pow(p2.z - p.z, 2));

					}
					//caso 2 : horario v3 e anti horario v2 - P3P2
					else if (!antihorarioV3 && antihorarioV2)
					{
						pcl::PointXYZ r = ProdutoVetorial(ProdutoVetorial(p_p3, p_p2), p2p3);
						double cosAngle_r = (p_p2.x*r.x + p_p2.y*r.y + p_p1.z*r.z) / (sqrt(pow(p_p2.x, 2) + pow(p_p2.y, 2) + pow(p_p2.z, 2)) * sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2)));
						double length_r = sqrt(pow(p_p2.x, 2) + pow(p_p2.y, 2) + pow(p_p2.z, 2)) * cosAngle_r;

						pcl::PointXYZ p_p__;
						p_p__.x = length_r * r.x / sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2));
						p_p__.y = length_r * r.y / sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2));
						p_p__.z = length_r * r.z / sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2));

						pcl::PointXYZ p__;
						p__.x = p_.x + p_p__.x;
						p__.y = p_.y + p_p__.y;
						p__.z = p_.z + p_p__.z;

						ponto = p__;

						double t = sqrt(pow(p__.x - p2.x, 2) + pow(p__.y - p2.y, 2) + pow(p__.z - p2.z, 2)) / sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2) + pow(p3.z - p2.z, 2));

						if ((t >= 0) && (t <= 1))
							dist = sqrt(pow(length_r, 2) + pow(lenp0, 2));
						else if (t < 0)
							dist = sqrt(pow(p2.x - p.x, 2) + pow(p2.y - p.y, 2) + pow(p2.z - p.z, 2));
						else if (t > 1)
							dist = sqrt(pow(p3.x - p.x, 2) + pow(p3.y - p.y, 2) + pow(p3.z - p.z, 2));
					}
					//caso 3 : horario v1 e anti horario v3 - P1P3
					//else if (!antihorarioV1 && antihorarioV3)
					else
					{
						pcl::PointXYZ r = ProdutoVetorial(ProdutoVetorial(p_p1, p_p3), p3p1);
						double cosAngle_r = (p_p3.x*r.x + p_p3.y*r.y + p_p1.z*r.z) / (sqrt(pow(p_p3.x, 2) + pow(p_p3.y, 2) + pow(p_p3.z, 2)) * sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2)));
						double length_r = sqrt(pow(p_p3.x, 2) + pow(p_p3.y, 2) + pow(p_p3.z, 2)) * cosAngle_r;

						pcl::PointXYZ p_p__;
						p_p__.x = length_r * r.x / sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2));
						p_p__.y = length_r * r.y / sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2));
						p_p__.z = length_r * r.z / sqrt(pow(r.x, 2) + pow(r.y, 2) + pow(r.z, 2));

						pcl::PointXYZ p__;
						p__.x = p_.x + p_p__.x;
						p__.y = p_.y + p_p__.y;
						p__.z = p_.z + p_p__.z;

						ponto = p__;

						double t = sqrt(pow(p__.x - p3.x, 2) + pow(p__.y - p3.y, 2) + pow(p__.z - p3.z, 2)) / sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2) + pow(p1.z - p3.z, 2));

						if ((t >= 0) && (t <= 1))
							dist = sqrt(pow(length_r, 2) + pow(lenp0, 2));
						else if (t < 0)
							dist = sqrt(pow(p3.x - p.x, 2) + pow(p3.y - p.y, 2) + pow(p3.z - p.z, 2));
						else if (t > 1)
							dist = sqrt(pow(p1.x - p.x, 2) + pow(p1.y - p.y, 2) + pow(p1.z - p.z, 2));
					}
				}

				if (dist < menorDistancia)
				{
					pontoProximo = ponto;
					closestFace = k;
					menorDistancia = dist;
				}
			}

			//face (triangulo) onde o ponto esta contido, ou mais proximo
			std::vector<unsigned int> f = blendShapeResult->faces[closestFace].vertices;
			pcl::PointXYZRGB p1 = blendShapeResult->cloud->at(f[0]);
			pcl::PointXYZRGB p2 = blendShapeResult->cloud->at(f[1]);
			pcl::PointXYZRGB p3 = blendShapeResult->cloud->at(f[2]);

			//baricentricas do ponto encontrado
			Eigen::Matrix<double, 3, 3> matrixPontos;
			matrixPontos(0, 0) = p1.x;	matrixPontos(0, 1) = p1.y;	matrixPontos(0, 2) = p1.z;
			matrixPontos(1, 0) = p2.x;	matrixPontos(1, 1) = p2.y;	matrixPontos(1, 2) = p2.z;
			matrixPontos(2, 0) = p3.x;	matrixPontos(2, 1) = p3.y;	matrixPontos(2, 2) = p3.z;

			Eigen::Matrix<double, 1, 3> matrixPonto;
			matrixPonto(0, 0) = pontoProximo.x;
			matrixPonto(0, 1) = pontoProximo.y;
			matrixPonto(0, 2) = pontoProximo.z;

			Eigen::Matrix<double, 1, 3> matrixAlphas = matrixPonto * matrixPontos.inverse();
			{
				double a1 = matrixAlphas(0, 0);
				double a2 = matrixAlphas(0, 1);
				double a3 = matrixAlphas(0, 2);

				//face (triangulo) correspondente no resultado da face do kinect - PCA
				std::vector<unsigned int> fM = faceMesh->faces[closestFace].vertices;
				pcl::PointXYZRGB p1 = faceMesh->cloud->at(fM[0]);
				pcl::PointXYZRGB p2 = faceMesh->cloud->at(fM[1]);
				pcl::PointXYZRGB p3 = faceMesh->cloud->at(fM[2]);

				pcl::PointXYZ ponto;
				ponto.x = a1 * p1.x + a2 * p2.x + a3 * p3.x;
				ponto.y = a1 * p1.y + a2 * p2.y + a3 * p3.y;
				ponto.z = a1 * p1.z + a2 * p2.z + a3 * p3.z;

				// se estiver muito fora do triangulo, e calculado e utilizado o ponto mais proximo ao triangulo 
				if (a1 > 2 || a2 > 2 || a3 > 2 || a1 < -2 || a2 < -2 || a3 < -2)
				{
					NuvemBlendShapeFacePCA->points[i].x = 0;
					NuvemBlendShapeFacePCA->points[i].y = 0;
					NuvemBlendShapeFacePCA->points[i].z = 0;

					pontosFora.push_back(ponto);
					i_pontosFora.push_back(i);
				}
				else
				{
					NuvemBlendShapeFacePCA->points[i].x = ponto.x;
					NuvemBlendShapeFacePCA->points[i].y = ponto.y;
					NuvemBlendShapeFacePCA->points[i].z = ponto.z;
				}
			}
		}

		//so a borda praticamente que fica muito fora, suavizando com o codigo abaixo
		for (size_t k = 0; k < pontosFora.size(); k++)
		{
			int i = i_pontosFora[k];
			pcl::PointXYZ ponto = pontosFora[k];

			std::vector<int> indices(1);
			std::vector<float> sqr_distances(1);

			pcl::KdTreeFLANN<pcl::PointXYZ> tree_a;
			tree_a.setInputCloud(NuvemBlendShapeFacePCA);
			tree_a.nearestKSearch(ponto, 1, indices, sqr_distances);

			NuvemBlendShapeFacePCA->points[i].x = NuvemBlendShapeFacePCA->points[indices[0]].x;
			NuvemBlendShapeFacePCA->points[i].y = NuvemBlendShapeFacePCA->points[indices[0]].y;
			NuvemBlendShapeFacePCA->points[i].z = NuvemBlendShapeFacePCA->points[indices[0]].z;
		}

		toPCLPointCloud2(*NuvemBlendShapeFacePCA, blendShape.BlendShape.cloud);

		pcl::io::savePLYFile("Temp/BlendShape_Original.ply", blendShape.BlendShapeOriginal);
		pcl::io::savePLYFile("Temp/BlendShape_Calibrado.ply", blendShape.BlendShape);

		//Calcula mascara
		blendShape.CriaMascara();
		pcl::io::savePLYFile("Temp/mask.ply", *blendShape.NuvemMascara);

		clock_t t1 = clock();
		printf("\n Mascara : %.f\n", ((double)(t1 - t0) / (CLOCKS_PER_SEC / 1000)));

		calibrado = true;
	}
	catch (std::exception& e)
	{
		cout << e.what() << endl;
		calibrado = false;
	}
}

void TProjeto::AjustaDistanciaOlhos()
{
	//from Camera
	//OlhoE - 36 - x // 37 - y
	int xEsquerdo = width - (tracker.PontosRosto[36 + shiftPontos].x + tracker.PontosRosto[39 + shiftPontos].x) / 2;
	int yEsquerdo = (tracker.PontosRosto[37 + shiftPontos].y + tracker.PontosRosto[41 + shiftPontos].y) / 2;
	pcl::PointXYZRGB pontoKinectOlhoEsquerdo = nuvem.RetornaPontoKinectfrom2D(xEsquerdo, yEsquerdo);

	//OlhoD - 42 - x // 43 - y
	int xDireito = width - (tracker.PontosRosto[42 + shiftPontos].x + tracker.PontosRosto[45 + shiftPontos].x) / 2;
	int yDireito = (tracker.PontosRosto[43 + shiftPontos].y + tracker.PontosRosto[47 + shiftPontos].y) / 2;
	pcl::PointXYZRGB pontoKinectOlhoDireito = nuvem.RetornaPontoKinectfrom2D(xDireito, yDireito);

	nuvem.DistanciaOlhos = std::sqrt(
		std::pow((pontoKinectOlhoDireito.x - pontoKinectOlhoEsquerdo.x), 2) +
		std::pow((pontoKinectOlhoDireito.y - pontoKinectOlhoEsquerdo.y), 2) +
		std::pow((pontoKinectOlhoDireito.z - pontoKinectOlhoEsquerdo.z), 2));
}

void TProjeto::AjustaEscala()
{
	AjustaDistanciaOlhos();
	blendShape.CalculaPontosRosto();

	bool first = escala == 0;

	escala = nuvem.DistanciaOlhos / blendShape.DistanciaOlhos;

	if (first || (escala > 1.00))
		blendShape.AjustaEscala(escala);
}

Eigen::Matrix4f TProjeto::ICP()
{
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(blendShape.NuvemMascara);
	icp.setInputTarget(nuvem.FaceKinect);
	pcl::PointCloud<pcl::PointXYZRGB> Final;

	icp.setMaxCorrespondenceDistance(0.05);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1);

	icp.align(Final);

	Eigen::Matrix4f matrix = icp.getFinalTransformation();

	pcl::transformPointCloud(*blendShape.NuvemMascara, *blendShape.NuvemMascara, matrix);
	return matrix;
}

void TProjeto::Renderiza(std::string filename, NuvemRGB::Ptr entrada)
{
	clock_t t0 = clock();

	int numExpressoes = blendShape.NumeroExpressoes;
	column_vector starting_point(numExpressoes);
	for (int i = 0; i < numExpressoes; i++)
		starting_point(i) = 0;

	TOtimizador Otimizador;
	Otimizador.blendshape = blendShape;
	Otimizador.FaceKinect = entrada;
	Otimizador.Calcula(starting_point);

	try
	{
		pcl::PolygonMesh vazio;
		pcl::io::loadPLYFile("../BlendShapes/vazio.ply", vazio);

		std::string filename1 = "Render/Original/" + filename + ".ply";
		std::remove(filename1.c_str());
		if (pcl::io::savePLYFile(filename1, Otimizador.BlendShapeOriginal) < 0)
			pcl::io::savePLYFile(filename1, vazio);

		std::string filename2 = "Render/Modificado/" + filename + ".ply";
		std::remove(filename2.c_str());
		if (pcl::io::savePLYFile(filename2, Otimizador.BlendShapeModificado) < 0)
			pcl::io::savePLYFile(filename1, vazio);
	}
	catch (std::exception&)
	{
		//do nothing
	}

	clock_t t1 = clock();
	printf("\nFrame : %.f\n", ((double)(t1 - t0) / (CLOCKS_PER_SEC / 1000)));

	mutexProcessos.lock();
	nProcessos--;
	mutexProcessos.unlock();
}

void TProjeto::Renderizacao()
{
	bool escalado = false;

	if (!calibrado)
	{
		Calibrar(nuvem.FaceKinect);
		calibrado = true;
	}

	std::vector<boost::filesystem::path> v;
	std::copy(boost::filesystem::directory_iterator("Input/"), boost::filesystem::directory_iterator(), std::back_inserter(v));
	std::sort(v.begin(), v.end(), sort_functor());

	int maxProcessos = 1;
	mutexProcessos.lock();
	nProcessos = 0;
	mutexProcessos.unlock();

	for (std::vector<boost::filesystem::path>::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
	{
		while (true)
		{
			mutexProcessos.lock();
			int x = nProcessos;
			mutexProcessos.unlock();

			if (x < maxProcessos)
				break;

			boost::this_thread::sleep(boost::posix_time::millisec(1000));
		}

		std::ostringstream ss;
		ss.str(std::string());
		ss.clear();
		ss << *it;
		std::cout << "Processando: " << *it << std::endl;

		pcl::PCDReader r;
		r.read(ss.str().substr(1, ss.str().length() - 2), *nuvem.CacheKinect);

		std::string filename = ss.str().substr(7, ss.str().length() - 12);

		if (nuvem.CacheKinect->size()) //Nuvem carregada
		{
			clock_t t0 = clock();

			for (int h = 0; h < ImagemKinectCache.rows; h++)
			{
				for (int w = 0; w < ImagemKinectCache.cols; w++)
				{
					pcl::PointXYZRGB point = nuvem.CacheKinect->at(w, h);
					Eigen::Vector3i rgb = point.getRGBVector3i();
					ImagemKinectCache.at<cv::Vec3b>(h, w)[0] = rgb[0];
					ImagemKinectCache.at<cv::Vec3b>(h, w)[1] = rgb[1];
					ImagemKinectCache.at<cv::Vec3b>(h, w)[2] = rgb[2];
				}
			}

			cv::Mat aux;
			cv::cvtColor(ImagemKinectCache, aux, CV_RGB2GRAY);
			cv::Mat ImagemGray;
			cv::flip(aux, ImagemGray, 1);

			//caso FaceTracker com sucesso
			tracker.FaceTracker.FrameReset();
			if (tracker.RastreiaRosto(ImagemGray))
			{
				nuvem.CarregaFace(tracker.PontosRosto, true);

				if (!escalado)
				{
					AjustaEscala();
					escalado = true;
				}

				NuvemXYZ::Ptr NuvemBlendShape(new NuvemXYZ());
				fromPCLPointCloud2(blendShape.BlendShape.cloud, *NuvemBlendShape);
				Eigen::Matrix4f MatrizSVD = PreAlinhamentoSVD();
				pcl::transformPointCloud(*blendShape.NuvemMascara, *blendShape.NuvemMascara, MatrizSVD);
				Eigen::Matrix4f MatrizICP = ICP();

				pcl::transformPointCloud(*NuvemBlendShape, *NuvemBlendShape, MatrizSVD);
				pcl::transformPointCloud(*NuvemBlendShape, *NuvemBlendShape, MatrizICP);
				toPCLPointCloud2(*NuvemBlendShape, blendShape.BlendShape.cloud);

				NuvemXYZ::Ptr NuvemBlendShapeOriginal(new NuvemXYZ());
				fromPCLPointCloud2(blendShape.BlendShapeOriginal.cloud, *NuvemBlendShapeOriginal);
				pcl::transformPointCloud(*NuvemBlendShapeOriginal, *NuvemBlendShapeOriginal, MatrizSVD);
				pcl::transformPointCloud(*NuvemBlendShapeOriginal, *NuvemBlendShapeOriginal, MatrizICP);
				toPCLPointCloud2(*NuvemBlendShapeOriginal, blendShape.BlendShapeOriginal.cloud);

				clock_t t1 = clock();
				printf("\nAlinhamento : %.f\n", ((double)(t1 - t0) / (CLOCKS_PER_SEC / 1000)));

				boost::thread(boost::bind(&TProjeto::Renderiza, this, filename, nuvem.FaceKinect));

				mutexProcessos.lock();
				nProcessos++;
				mutexProcessos.unlock();
			}
		}
	}
}

static unsigned fps = 0;
static double lastFPS = clock() / (CLOCKS_PER_SEC / 1000);

#define FPS_CALC(_WHAT_) \
do \
{\
	++fps;\
	if ((clock() / (CLOCKS_PER_SEC / 1000)) - lastFPS >= 1000)\
	{\
		double now = pcl::getTime();\
		std::cout << _WHAT_ << " : " << (double(fps)) << " FPS" << std::endl;\
		fps = 0;\
		lastFPS = clock() / (CLOCKS_PER_SEC / 1000);\
	}\
} while (false)

void TProjeto::Run(int Tipo)
{
	Visualizer.setBackgroundColor(0, 0, 0);
	Visualizer.setCameraPosition(0.0043975, -0.0517195, 0.341018, -0.0253505, -0.0013949, 0.802403, -0.0180372, 0.993816, -0.109561);
	Visualizer.setCameraFieldOfView(0.523599);
	Visualizer.setCameraClipDistances(0.355952, 0.603558);
	Visualizer.setPosition(0, 0);
	Visualizer.setSize(640, 480);

	if (Tipo == 0) //Grabber
	{
		// tela dividida em viewports
		int v1(1); Visualizer.createViewPort(0.00, 0.50, 0.50, 1.00, v1);	MontaViewport(Visualizer, "Entrada Kinect", 1, 0.3, 0.3, 0.3);
		int v2(2); Visualizer.createViewPort(0.00, 0.00, 0.50, 0.50, v2);	MontaViewport(Visualizer, "Face Recortada", 2, 0.3, 0.3, 0.3);
		int v3(3); Visualizer.createViewPort(0.50, 0.00, 1.00, 1.00, v3);	MontaViewport(Visualizer, "Saida (Resultado)", 3, 0.3, 0.3, 0.3);

		//loop until quit (i.e user presses ESC)
		while (!Visualizer.wasStopped())
		{
			//Visualizer.removeAllPointClouds();
			//Visualizer.removeAllShapes();
			try
			{
				//Nuvem salva em disco
				if (grabber == 0)
				{
					playermutex.lock();
					if (playerCache.size() > 0)
					{
						nuvem.CacheKinect = playerCache[0];
						playerCache.erase(playerCache.begin());
					}
					else
						nuvem.CacheKinect = NULL;
					playermutex.unlock();
				}

				if (nuvem.CacheKinect != NULL)
				{
					if (nuvem.CacheKinect->size()) //Nuvem carregada
					{
						for (int h = 0; h < ImagemKinectCache.rows; h++)
						{
							for (int w = 0; w < ImagemKinectCache.cols; w++)
							{
								pcl::PointXYZRGB point = nuvem.CacheKinect->at(w, h);
								Eigen::Vector3i rgb = point.getRGBVector3i();
								ImagemKinectCache.at<cv::Vec3b>(h, w)[0] = rgb[2];
								ImagemKinectCache.at<cv::Vec3b>(h, w)[1] = rgb[1];
								ImagemKinectCache.at<cv::Vec3b>(h, w)[2] = rgb[0];
							}
						}

						cv::Mat aux;
						cv::cvtColor(ImagemKinectCache, aux, CV_RGB2GRAY);
						cv::Mat ImagemGray;
						cv::flip(aux, ImagemGray, 1);

						//cv::flip(ImagemKinectCache, ImagemKinectCache, 1);
						//imshow("", ImagemKinectCache);

						//caso FaceTracker com sucesso
						if (tracker.RastreiaRosto(ImagemGray))
						{
							nuvem.CarregaFace(tracker.PontosRosto, calibrado);

							if (!calibrado)
							{
								AjustaEscala();

								NuvemXYZ::Ptr NuvemBlendShape(new NuvemXYZ());
								fromPCLPointCloud2(blendShape.BlendShape.cloud, *NuvemBlendShape);
								Eigen::Matrix4f MatrizSVD = PreAlinhamentoSVD();
								pcl::transformPointCloud(*NuvemBlendShape, *NuvemBlendShape, MatrizSVD);
								toPCLPointCloud2(*NuvemBlendShape, blendShape.BlendShape.cloud);
							}
							else //já calibrado
							{
								NuvemXYZ::Ptr NuvemBlendShape(new NuvemXYZ());
								fromPCLPointCloud2(blendShape.BlendShape.cloud, *NuvemBlendShape);
								Eigen::Matrix4f MatrizSVD = PreAlinhamentoSVD();
								pcl::transformPointCloud(*blendShape.NuvemMascara, *blendShape.NuvemMascara, MatrizSVD);
								Eigen::Matrix4f MatrizICP = ICP();

								pcl::transformPointCloud(*NuvemBlendShape, *NuvemBlendShape, MatrizSVD);
								pcl::transformPointCloud(*NuvemBlendShape, *NuvemBlendShape, MatrizICP);
								toPCLPointCloud2(*NuvemBlendShape, blendShape.BlendShape.cloud);

								NuvemXYZ::Ptr NuvemBlendShapeOriginal(new NuvemXYZ());
								fromPCLPointCloud2(blendShape.BlendShapeOriginal.cloud, *NuvemBlendShapeOriginal);
								pcl::transformPointCloud(*NuvemBlendShapeOriginal, *NuvemBlendShapeOriginal, MatrizSVD);
								pcl::transformPointCloud(*NuvemBlendShapeOriginal, *NuvemBlendShapeOriginal, MatrizICP);
								toPCLPointCloud2(*NuvemBlendShapeOriginal, blendShape.BlendShapeOriginal.cloud);
							}

							//mostra nova caixa com imagem 2D
							//imshow("", ImagemGray);

							if (!Visualizer.updatePointCloud(nuvem.FaceKinect, "FaceKinect"))
								Visualizer.addPointCloud(nuvem.FaceKinect, "FaceKinect", 2);

							for (int i = 0; i < 3; ++i)
							{
								pcl::PointXYZRGB p;
								p.x = nuvem.PontosRostoKinect->points[i].x;
								p.y = nuvem.PontosRostoKinect->points[i].y;
								p.z = nuvem.PontosRostoKinect->points[i].z;

								if (p.x != 0 || p.y != 0 || p.z != 0)
								{
									int r = 0, g = 255, b = 0;

									std::ostringstream ss;
									ss << "FaceKinect_landmark99" << i << "_vp";
									float radius = 0.001;
									if (!Visualizer.updateSphere(p, radius, r, g, b, ss.str()))
										Visualizer.addSphere(p, radius, ss.str(), 2);
								}
							}

							int n = 19;
							for (int i = 0; i < n; ++i)
							{
								pcl::PointXYZRGB p = nuvem.RetornaPontoKinectfrom2D(width - tracker.PontosRosto[i].x, tracker.PontosRosto[i].y);

								if (p.x != 0 || p.y != 0 || p.z != 0)
								{
									int r = 0, g = 255, b = 0;

									std::ostringstream ss;
									ss << "FaceKinect_landmark" << i << "_vp";
									float radius = 0.001;
									if (!Visualizer.updateSphere(p, radius, r, g, b, ss.str()))
										Visualizer.addSphere(p, radius, ss.str(), 1);

									if (n > 1)
									{
										ss << i;
										pcl::PointXYZRGB p0;

										if (i == 0)
											p0 = nuvem.RetornaPontoKinectfrom2D(width - tracker.PontosRosto[i + 1].x, tracker.PontosRosto[i + 1].y);
										else
											p0 = nuvem.RetornaPontoKinectfrom2D(width - tracker.PontosRosto[i - 1].x, tracker.PontosRosto[i - 1].y);

										Visualizer.removeShape(ss.str(), 1);
										Visualizer.addLine(p0, p, r, g, b, ss.str(), 1);
									}
								}
							}

							pcl::PointXYZRGB p0 = nuvem.RetornaPontoKinectfrom2D(width - tracker.PontosRosto[0].x, tracker.PontosRosto[0].y);
							pcl::PointXYZRGB pN = nuvem.RetornaPontoKinectfrom2D(width - tracker.PontosRosto[n - 1].x, tracker.PontosRosto[n - 1].y);

							Visualizer.removeShape("linha_final", 1);
							Visualizer.addLine(p0, pN, 0, 255, 0, "linha_final", 1);

						}
						else
							tracker.FaceTracker.FrameReset();
					}

					if (!Visualizer.updatePointCloud(nuvem.CacheKinect, "CacheKinect"))
						Visualizer.addPointCloud(nuvem.CacheKinect, "CacheKinect", 1);

					if (!Visualizer.updatePointCloud(nuvem.CacheKinect, "CacheKinect3"))
						Visualizer.addPointCloud(nuvem.CacheKinect, "CacheKinect3", 3);

					if (!Visualizer.updatePolygonMesh(blendShape.BlendShape, "BlendShape"))
						Visualizer.addPolygonMesh(blendShape.BlendShape, "BlendShape", 3);
				}

				//Atualiza Visualizer
				Visualizer.spinOnce();
			}
			catch (std::exception& e)
			{
				cout << e.what() << endl;
				continue;
			}
		}

		if (grabber != 0)
			grabber->stop();
	}
	else if (Tipo == 1) //Player
	{
		int v1(1); Visualizer.createViewPort(0.00, 0.00, 1.00, 1.00, v1);
		MontaViewport(Visualizer, "Modificado", 1, 0.3, 0.3, 0.3);

		pcl::PCDReader r;
		std::ostringstream ss;
		std::vector<boost::filesystem::path> v;
		std::copy(boost::filesystem::directory_iterator("Input/"), boost::filesystem::directory_iterator(), std::back_inserter(v));
		std::sort(v.begin(), v.end(), sort_functor());

		NuvemRGB::Ptr cache;
		cache.reset(new NuvemRGB());
		cache->is_dense = true;

		while (!Visualizer.wasStopped())
		{
			clock_t t0 = clock();
			if (recorder)
			{
				for (std::vector<boost::filesystem::path>::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
				{
					t0 = clock();

					ss.str(std::string());
					ss.clear();
					ss << *it;

					if (r.read(ss.str().substr(1, ss.str().length() - 2), *cache) != -1)
					{
						if (!Visualizer.updatePointCloud(cache, "Entrada"))
							Visualizer.addPointCloud(cache, "Entrada", 1);
					}

					for (int h = 0; h < ImagemKinectCache.rows; h++)
					{
						for (int w = 0; w < ImagemKinectCache.cols; w++)
						{
							pcl::PointXYZRGB point = cache->at(w, h);
							Eigen::Vector3i rgb = point.getRGBVector3i();
							ImagemKinectCache.at<cv::Vec3b>(h, w)[0] = rgb[2];
							ImagemKinectCache.at<cv::Vec3b>(h, w)[1] = rgb[1];
							ImagemKinectCache.at<cv::Vec3b>(h, w)[2] = rgb[0];
						}
					}

					cv::flip(ImagemKinectCache, ImagemKinectCache, 1);
					imshow("", ImagemKinectCache);

					pcl::PolygonMesh modificado;
					if (FileExists("Render/Modificado/" + ss.str().substr(7, ss.str().length() - 12) + ".ply")
						&& (pcl::io::loadPLYFile("Render/Modificado/" + ss.str().substr(7, ss.str().length() - 12) + ".ply", modificado) != -1))
					{
						if (modificado.polygons.size() > 0)
						{
							if (!Visualizer.updatePolygonMesh(modificado, "Modificado"))
								Visualizer.addPolygonMesh(modificado, "Modificado", 1);
						}
						else
							Visualizer.removePolygonMesh("Modificado");
					}
					else
						Visualizer.removePolygonMesh("Modificado");

					//pcl::PolygonMesh original;
					//if (FileExists("Render/Original/" + ss.str().substr(7, ss.str().length() - 12) + ".ply")
					//	&& (pcl::io::loadPLYFile("Render/Original/" + ss.str().substr(7, ss.str().length() - 12) + ".ply", original) != -1))
					//{
					//	if (original.polygons.size() > 0)
					//	{
					//		if (!Visualizer.updatePolygonMesh(original, "Resultado"))
					//			Visualizer.addPolygonMesh(original, "Resultado", 3);
					//	}
					//	else
					//		Visualizer.removePolygonMesh("Resultado");
					//}
					//else
					//	Visualizer.removePolygonMesh("Resultado");

					clock_t t1 = clock();
					double t = 100 - (double)(t1 - t0) / (CLOCKS_PER_SEC / 1000);

					Visualizer.spinOnce();
					if (t > 0)
						boost::this_thread::sleep(boost::posix_time::milliseconds(t));

					FPS_CALC("render");
				}
			}
			else
			{
				clock_t t1 = clock();
				double t = 100 - (double)(t1 - t0) / (CLOCKS_PER_SEC / 1000);
				Visualizer.spinOnce();
				boost::this_thread::sleep(boost::posix_time::milliseconds(t));
				FPS_CALC("render");
			}
		}
	}
	else if (Tipo == 11) //Player
	{
		int v1(1); Visualizer.createViewPort(0.00, 0.00, 1.00, 1.00, v1);
		MontaViewport(Visualizer, "Resultado", 1, 0.3, 0.3, 0.3);

		pcl::PCDReader r;
		std::ostringstream ss;
		std::vector<boost::filesystem::path> v;
		std::copy(boost::filesystem::directory_iterator("Input/"), boost::filesystem::directory_iterator(), std::back_inserter(v));
		std::sort(v.begin(), v.end(), sort_functor());

		NuvemRGB::Ptr cache;
		cache.reset(new NuvemRGB());
		cache->is_dense = true;

		while (!Visualizer.wasStopped())
		{
			clock_t t0 = clock();
			if (recorder)
			{
				for (std::vector<boost::filesystem::path>::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
				{
					t0 = clock();

					ss.str(std::string());
					ss.clear();
					ss << *it;

					if (r.read(ss.str().substr(1, ss.str().length() - 2), *cache) != -1)
					{
						if (!Visualizer.updatePointCloud(cache, "Entrada"))
							Visualizer.addPointCloud(cache, "Entrada", 1);
					}

					for (int h = 0; h < ImagemKinectCache.rows; h++)
					{
						for (int w = 0; w < ImagemKinectCache.cols; w++)
						{
							pcl::PointXYZRGB point = cache->at(w, h);
							Eigen::Vector3i rgb = point.getRGBVector3i();
							ImagemKinectCache.at<cv::Vec3b>(h, w)[0] = rgb[2];
							ImagemKinectCache.at<cv::Vec3b>(h, w)[1] = rgb[1];
							ImagemKinectCache.at<cv::Vec3b>(h, w)[2] = rgb[0];
						}
					}

					cv::flip(ImagemKinectCache, ImagemKinectCache, 1);
					imshow("", ImagemKinectCache);

					pcl::PolygonMesh original;
					if (FileExists("Render/Original/" + ss.str().substr(7, ss.str().length() - 12) + ".ply")
						&& (pcl::io::loadPLYFile("Render/Original/" + ss.str().substr(7, ss.str().length() - 12) + ".ply", original) != -1))
					{
						if (original.polygons.size() > 0)
						{
							if (!Visualizer.updatePolygonMesh(original, "Resultado"))
								Visualizer.addPolygonMesh(original, "Resultado", 1);
						}
						else
							Visualizer.removePolygonMesh("Resultado");
					}
					else
						Visualizer.removePolygonMesh("Resultado");

					clock_t t1 = clock();
					double t = 100 - (double)(t1 - t0) / (CLOCKS_PER_SEC / 1000);

					Visualizer.spinOnce();
					if (t > 0)
						boost::this_thread::sleep(boost::posix_time::milliseconds(t));

					FPS_CALC("render");
				}
			}
			else
			{
				clock_t t1 = clock();
				double t = 100 - (double)(t1 - t0) / (CLOCKS_PER_SEC / 1000);
				Visualizer.spinOnce();
				boost::this_thread::sleep(boost::posix_time::milliseconds(t));
				FPS_CALC("render");
			}
		}
	}
	else if (Tipo == 12) //Player
	{
		int v1(1); Visualizer.createViewPort(0.00, 0.00, 1.00, 1.00, v1);
		MontaViewport(Visualizer, "Avatar", 1, 0.3, 0.3, 0.3);

		pcl::PCDReader r;
		std::ostringstream ss;
		std::vector<boost::filesystem::path> v;
		std::copy(boost::filesystem::directory_iterator("Input/"), boost::filesystem::directory_iterator(), std::back_inserter(v));
		std::sort(v.begin(), v.end(), sort_functor());

		NuvemRGB::Ptr cache;
		cache.reset(new NuvemRGB());
		cache->is_dense = true;

		while (!Visualizer.wasStopped())
		{
			clock_t t0 = clock();
			if (recorder)
			{
				for (std::vector<boost::filesystem::path>::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
				{
					t0 = clock();

					ss.str(std::string());
					ss.clear();
					ss << *it;

					pcl::PolygonMesh original;
					if (FileExists("Render/Original/" + ss.str().substr(7, ss.str().length() - 12) + ".ply")
						&& (pcl::io::loadPLYFile("Render/Original/" + ss.str().substr(7, ss.str().length() - 12) + ".ply", original) != -1))
					{
						if (original.polygons.size() > 0)
						{
							if (!Visualizer.updatePolygonMesh(original, "Resultado"))
								Visualizer.addPolygonMesh(original, "Resultado", 1);
						}
						else
							Visualizer.removePolygonMesh("Resultado");
					}
					else
						Visualizer.removePolygonMesh("Resultado");

					clock_t t1 = clock();
					double t = 100 - (double)(t1 - t0) / (CLOCKS_PER_SEC / 1000);

					Visualizer.spinOnce();
					if (t > 0)
						boost::this_thread::sleep(boost::posix_time::milliseconds(t));

					FPS_CALC("render");
				}
			}
			else
			{
				clock_t t1 = clock();
				double t = 100 - (double)(t1 - t0) / (CLOCKS_PER_SEC / 1000);
				Visualizer.spinOnce();
				boost::this_thread::sleep(boost::posix_time::milliseconds(t));
				FPS_CALC("render");
			}
		}
	}
	else if (Tipo == 2) //Recorder
	{
		if (grabber != 0)
		{
			std::ostringstream ss;

			time_t     now = time(0);
			struct tm  tstruct;
			char       buf[80];
			tstruct = *localtime(&now);
			strftime(buf, sizeof(buf), "%d%m%Y_%H_%M", &tstruct);
			ss << buf;

			rec_path = "Recorder/" + ss.str() + "/";

			boost::filesystem::path dir(rec_path);
			if (boost::filesystem::create_directory(dir))
				recorder_frame = 0;

			int v1(1); Visualizer.createViewPort(0.00, 0.00, 1.00, 1.00, v1);	MontaViewport(Visualizer, "Recorder " + rec_path, 1, 0.3, 0.3, 0.3);

			while (!Visualizer.wasStopped())
			{
				if (!Visualizer.updatePointCloud(nuvem.CacheKinect, "CacheKinect"))
					Visualizer.addPointCloud(nuvem.CacheKinect, "CacheKinect", 1);

				if (recorder)
				{
					recorder_frame++;

					std::ostringstream ss;
					ss.str(std::string());
					ss.clear();
					ss << recorder_frame;

					pcl::PCDWriter w;
					w.writeBinaryCompressed(rec_path + ss.str() + ".pcd", *nuvem.CacheKinect);

					if (!Visualizer.updateText("Gravando " + ss.str() + " frames", 5, 400, 15, 1.0, 0.0, 0.0, "recorder_text"))
						Visualizer.addText("Gravando" + ss.str() + " frames", 5, 400, 15, 1.0, 0.0, 0.0, "recorder_text", 1);
				}

				//Atualiza Visualizer
				Visualizer.spinOnce();
			}
		}
	}
	else if (Tipo == 3) //renderiza video
	{
		pcl::io::loadPLYFile("Temp/BlendShape_Calibrado.ply", blendShape.BlendShape);
		pcl::io::loadPLYFile("Temp/BlendShape_Original.ply", blendShape.BlendShapeOriginal);

		//calcula mascara
		blendShape.CriaMascara();

		calibrado = true;

		Renderizacao();
	}
};
