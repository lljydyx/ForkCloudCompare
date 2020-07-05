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
#include <set>
#include <stdlib.h>

//GDAL
#include <gdal.h>
#include <ogrsf_frmts.h>

//local
#include "ccPointCloud.h"
#include "mainwindow.h"
#include <ccScalarField.h>

//eigen3
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
using namespace Eigen;

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
	int LayNumber;

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
		LayNumber = 0;
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
		LayNumber = P.LayNumber;

		return *this;
	}
};

class xjGrid
{
public:
	int pointCount;
	float maxX;
	float minX;
	float deltaX;
	float maxY;
	float minY;
	float deltaY;
	float maxZ;
	float minZ;
	float deltaZ;

	xjGrid::xjGrid(void)
	{
		pointCount = 0;
		maxX = -666;
		minX = 666;
		deltaX = 0;
		maxY = -666;
		minY = 666;
		deltaY = 0;
		maxZ = -666;
		minZ = 666;
		deltaZ = 0;
	}

	xjGrid &operator =(const xjGrid &Grid)
	{
		pointCount = Grid.pointCount;
		maxX = Grid.maxX;
		minX = Grid.minX;
		deltaX = Grid.deltaX;
		maxY = Grid.minX;
		minY = Grid.minY;
		deltaY = Grid.deltaY;
		maxZ = Grid.maxZ;
		minZ = Grid.minZ;
		deltaZ = Grid.deltaZ;

		return *this;
	}
};

struct xjLasParameter
{
	float maxX;
	float minX;
	float maxY;
	float minY;
	float maxZ;
	float minZ;

	float GSD;
	bool bVoxel;
	int sumRow;
	int sumCol;
	int sumLay;

	int desnoseCount;
	float thrSlope;
	float thrDeltaZ;
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

	/* denoising */
	void xjDenoising(QMultiHash<int, xjPoint>& mhGridding, xjLasParameter & parameter);

	/* collection of grid number */
	QMultiHash<int, int> xjGetGridNumber(const QMultiHash<int, xjPoint>& mhGridding, xjLasParameter & parameter);

	/* cluster */
	QList<QList<int>> xjCluster(const QMultiHash<int, int>& mhGridNumber, xjLasParameter & parameter);
	//interation
	void RegionGrowth(QList<int>& list, const int & cGridNumber, const int & sumCol, const QMultiHash<int, int>& mhGridNumber, QMultiHash<int, int>& mhKey);

	QMultiHash<int, xjGrid> xjGridProperty(const QMultiHash<int, xjPoint>& mhGrid, xjLasParameter & parameter);

	/* create ccPointCloud */
	void xjCreatePointCloud(const QMultiHash<int, xjPoint>& mhGridding, xjLasParameter & parameter, ccPointCloud * newCloud);

	/* 高程统计过滤 Elevation Statistical Filter */
	QList<xjPoint> xjElevationStatisticalFilter(const QList<xjPoint>& listP, const xjLasParameter &parameter);

	//获取行号和列号 get the row and column numbers based on the grid number
	void GetRowAndCol(int & row, int & col, const int & gridNumber, const int & sumCol);

	/* mean Z */
	float xjMeanValue(const QList<xjPoint>& listP);

	/* neighbourhood point */
	float xjNeighbourhoodPoint(const QMultiHash<int, xjPoint>& mhGridding, xjLasParameter &parameter, const int & gn, const int & layer);

	/* 特征值和特征向量 eigenvalue and eigenvector */
	int xjEigenValueVectorShape(const QList<xjPoint>& listP, double &includedAngle);
};

#endif // !XJ_ALGORITHM