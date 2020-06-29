#ifndef XJ_ALGORITHM
#define XJ_ALGORITHM

//system
#include <set>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <limits>
#include <iostream>

//GDAL
#include <gdal.h>
#include <ogrsf_frmts.h>

//local
#include "ccPointCloud.h"
#include "mainwindow.h"
#include <ccScalarField.h>

class xjAlgorithm
{
public:
	void ViewPartResult(ccPointCloud * cloud, const std::vector<CCVector3>& vPoint, const QString & resultName, const int & pointSize, const float & r, const float & g, const float & b);
	void ViewPartResultShp(const std::vector<CCVector3>& vPoint, const QString & xjName);
	uint xjGetIntensityByOTSU(ccPointCloud * cloud, const char * sf, const bool & bSmooth);
	std::vector<CCVector3> xjGetTrafficMarkingPoints(ccPointCloud * cloud, const char * sf, const uint & threshold, const bool & bCluster);
	double TwoDistancePointAndPoint(const CCVector3 & p1, const CCVector3 & p2);
	double xjGetCrossProduct(const CCVector3 & p1, const CCVector3 & p2, const CCVector3 & p0);
	std::vector<CCVector3> xjGetConvexHullByGrahamScan(std::vector<CCVector3> vPoint);
	double ComputePolygonArea(const std::vector<CCVector3>& points);
	double TwoDistancePointAndLine(const CCVector3 & p0, const CCVector3 & p1, const CCVector3 & p2);
	std::vector<CCVector3> xjMinimumBoundingRectangle(double & length, double & width, const std::vector<CCVector3>& vCHpoint);
};

#endif // !XJ_ALGORITHM