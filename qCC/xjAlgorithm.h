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

class xjPoint
{
public:
	double x;
	double y;
	double z;
	int Intensity;
	int PointSourceID;
	double GPStime;
	int Classification;
	int UserData;
	int r, g, b;
	int PID;

	int Location;
	int RowNumber;
	int ColNumber;

	xjPoint::xjPoint(void)
	{
		x = 0;
		y = 0;
		z = 0;
		Intensity = 0;
		PointSourceID = 0;
		GPStime = 0;
		Classification = 0;
		UserData = 0;
		r = 0;
		g = 0;
		b = 0;
		PID = -1;
		Location = 0;
		RowNumber = 0;
		ColNumber = 0;
	}


	bool operator ==(const xjPoint &P1) const
	{
		if ((x == P1.x) && (y == P1.y) && (z == P1.z))
			return true;
		else return false;
	}

	xjPoint &operator =(const xjPoint &P)
	{
		x = P.x;
		y = P.y;
		z = P.z;

		Intensity = P.Intensity;
		PointSourceID = P.PointSourceID;
		GPStime = P.GPStime;
		Classification = P.Classification;
		UserData = P.UserData;

		r = P.r;
		g = P.g;
		b = P.b;

		PID = P.PID;

		Location = P.Location;
		RowNumber = P.RowNumber;
		ColNumber = P.ColNumber;

		return *this;
	}

	void setXYZ(double X, double Y, double Z)
	{
		x = X; y = Y; z = Z;
	}

	void setIntensity(int intensity)
	{
		Intensity = intensity;
	}
	void setPointSourceId(int PSId)
	{
		PointSourceID = PSId;
	}
	void setGPStime(double gpstime)
	{
		GPStime = gpstime;
	}

	void setRGB(int R, int G, int B)
	{
		r = R; g = G; b = B;
	}

	void setClassification(int classification)
	{
		Classification = classification;
	}
	void setUserData(int userData)
	{
		UserData = userData;
	}
};

class xjGrid
{
public:
	int pointCount;
	xjPoint maxP;
	xjPoint minP;

	xjGrid::xjGrid(void)
	{
		pointCount = 0;
	}
};

struct xjLasParameter
{
	float GSD;
	int sumRow;
	int sumCol;
	int sumLay;
};

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
	
	
	QMultiHash<int, xjPoint> xjGridding(ccPointCloud * cloud, xjLasParameter &parameter);
	QMultiHash<int, xjGrid> xjGridProperty(const QMultiHash<int, xjPoint>& mhGrid, xjLasParameter & parameter);
};

#endif // !XJ_ALGORITHM