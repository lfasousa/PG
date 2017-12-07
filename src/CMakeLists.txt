cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ProjetoGraduacao)

INCLUDE(ConfigureANN.cmake)
INCLUDE(ConfigureCLAPACK.cmake)
INCLUDE(ConfigureITK.cmake)

find_package(PCL 1.8 REQUIRED COMPONENTS COMMON OCTREE IO KDTREE SEARCH SAMPLE_CONSENSUS FILTERS FEATURES SEGMENTATION VISUALIZATION SURFACE REGISTRATION KEYPOINTS TRACKING RECOGNITION APPS OUTOFCORE PEOPLE)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

FIND_PACKAGE( Boost 1.40 COMPONENTS REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

include_directories("C:/Program Files/Eigen/include/eigen3")
include_directories("C:/Program Files/flann/include")
include_directories("C:/Program Files/OpenNI/Include")
include_directories("C:/Program Files/Microsoft SDKs/Kinect/v1.8/inc")

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories( ${VTK_LIBRARY_DIRS})

include_directories(${FLANN_INCLUDE_DIRS})

include_directories("C:/Program Files/dlib/include")

link_directories("C:/Program Files/Boost/lib")
link_directories("C:/Program Files/Microsoft SDKs/Kinect/v1.8/lib/amd64")

include_directories ("${PROJECT_SOURCE_DIR}/include")
link_directories("${PROJECT_SOURCE_DIR}/lib")	

add_executable (
	ProjetoGraduacao main.cpp kinect_grabber.cpp Classe.Nuvem.cpp Classe.Projeto.cpp Classe.Otimizador.cpp Classe.BlendShape.cpp Classe.Tracker.cpp Classe.ModeloFace.cpp Classe.PCA.cpp include/CLM.cc include/FCheck.cc include/FDet.cc include/IO.cc include/Patch.cc include/PAW.cc include/PDM.cc include/Tracker.cc PCA/KDTree3.h PCA/KDTree3.cpp PCA/DataContainer.h PCA/Definitions.h PCA/FileHandler.h PCA/FileLoader.h PCA/FileWriter.h PCA/MathHelper.h PCA/MMProjectionCostFunction.h PCA/MultilinearModel.h PCA/MultilinearModelHandler.h PCA/RegistrationHelper.h PCA/VectorNX.h PCA/FileHandler.cpp PCA/FileLoader.cpp PCA/FileWriter.cpp PCA/MathHelper.cpp PCA/MMProjectionCostFunction.cpp PCA/MultilinearModel.cpp PCA/MultilinearModelHandler.cpp )

target_link_libraries (ProjetoGraduacao ${PCL_LIBRARIES})
target_link_libraries( ProjetoGraduacao ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( ProjetoGraduacao ${Boost_LIBRARIES} )
target_link_libraries( ProjetoGraduacao ${VTK_LIBRARIES})
target_link_libraries( ProjetoGraduacao "C:/Program Files/Microsoft SDKs/Kinect/v1.8/lib/amd64/Kinect10.lib")




