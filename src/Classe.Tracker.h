#pragma once
#ifndef __TRACKER_INCLUDED__
#define  __TRACKER_INCLUDED__

#include <pcl/point_types.h>

#include <FaceTracker/Tracker.h>

#include <boost/assign/list_of.hpp>

class TTracker
{
public:
	TTracker();

	static const char* traFile;

	FACETRACKER::Tracker FaceTracker;
	std::vector<pcl::PointXYZ> PontosRosto;

	bool RastreiaRosto(cv::Mat);

	std::vector<cv::Point> landmarks;
};

#endif