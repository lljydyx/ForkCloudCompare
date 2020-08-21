#include "xjAlgorithm.h"
class xjGrid;


/* �̱��� short marking: view part result: ccPointCloud*/
void xjAlgorithm::ViewPartResult(ccPointCloud *cloud, const std::vector<CCVector3> &vPoint, 
	const QString &resultName, const int &pointSize, const float &r, const float &g, const float &b)
{
	ccPointCloud *pcPR = new ccPointCloud(resultName);
	if (!pcPR->reserveThePointsTable(vPoint.size()))
	{
		ccLog::Error("No enough memory!!!");
		delete pcPR;
		pcPR=nullptr;
		return;
	}
	float x = 0, y = 0, z = 0;
	for (unsigned i = 0; i < vPoint.size(); ++i)
	{
		pcPR->addPoint(vPoint.at(i));
	}
	pcPR->invalidateBoundingBox();
	pcPR->setPointSize(pointSize);
	pcPR->colorize(r, g, b);
	pcPR->showColors(true);
	pcPR->setGlobalScale(cloud->getGlobalScale());
	pcPR->setGlobalShift(cloud->getGlobalShift());
	pcPR->setDisplay(cloud->getDisplay());
	pcPR->prepareDisplayForRefresh();
	pcPR->refreshDisplay();
	if (cloud->getParent())
		cloud->getParent()->addChild(pcPR);
	MainWindow::TheInstance()->addToDB(pcPR);
}
/* �̱��� short marking: view part result: shape file */
void xjAlgorithm::ViewPartResultShp(const std::vector<CCVector3> &vPoint, const QString &xjName)
{
	GDALAllRegister();
	OGRRegisterAll();
	const char *xjDriverName = "ESRI Shapefile";
	GDALDriver *xjDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(xjDriverName);
	if (xjDriver == NULL)
	{
		//QMessageBox::warning(0, ("prompt"), QString(xjDriverName) + "driver not available!!!");
		return;
	}

	QString xjPath = "F://MBR//";
	GDALDataset *xjDataset = xjDriver->Create(xjPath.toStdString().c_str(), 0, 0, 0, GDT_Unknown, NULL);
	if (xjDataset == NULL)
	{
		//QMessageBox::warning(0, ("prompt"), "Creation of output file failed!!!");
		return;
	}

	OGRLayer *xjLayer = xjDataset->CreateLayer(xjName.toStdString().c_str(), NULL, wkbPolygonZM, NULL);
	if (xjLayer == NULL)
	{
		//QMessageBox::warning(0, ("prompt"), "Layer creation failed!!!");
		return;
	}

	//����Ҫ��
	OGRFeature *xjFeature = OGRFeature::CreateFeature(xjLayer->GetLayerDefn());
	OGRLinearRing xjRing;
	int pointCount = vPoint.size();
	for (int i = 0; i < pointCount; i++)
	{
		xjRing.addPoint(vPoint.at(i).x, vPoint.at(i).y, vPoint.at(i).z);
	}
	xjRing.closeRings();//��β���غ��γɱպϻ�

	//ͼ�����Ҫ��
	OGRPolygon xjPolygon;
	xjPolygon.addRing(&xjRing);
	xjFeature->SetGeometry(&xjPolygon);

	//�ж�
	if (xjLayer->CreateFeature(xjFeature) != OGRERR_NONE)
	{
		//QMessageBox::warning(0, ("prompt"), "Failed to create feature in shapefile!!!");
		return;
	}
	OGRFeature::DestroyFeature(xjFeature);

	GDALClose(xjDataset);
}

/* �̱��� short marking: The returned threshold of intensity is obtained by OTSU. */
uint xjAlgorithm::xjGetIntensityByOTSU(ccPointCloud* cloud, const char *sf, const bool &bSmooth)
{
	uint thrIntensity = 1;

	/* 1 ǿ��ֱ��ͼ */
	uint histogramIntensity[65536] = { 0 };
	uint maxIntensity = 0, minIntensity = 666666;
	uint pcCount = cloud->size();
	ccScalarField* ccSF = static_cast<ccScalarField*>(cloud->getScalarField(cloud->getScalarFieldIndexByName(sf)));
	for (int i = 0; i < pcCount; i++)
	{
		uint vIntensity = ccSF->getValue(i);
		if (vIntensity > maxIntensity)
		{
			maxIntensity = vIntensity;
		}
		if (vIntensity < minIntensity)
		{
			minIntensity = vIntensity;
		}
		++histogramIntensity[vIntensity];
	}

	/* 2 �븽��2���Ҷ���ƽ������tֵӦȡ��С��ֵ */
	if (bSmooth)
	{
		for (int k = minIntensity; k <= maxIntensity; k++)
		{
			uint total = 0;
			for (int t = -2; t <= 2; t++)
			{
				int q = k + t;
				if (q < 0) //Խ�紦��
					q = 0;
				if (q > 255)
					q = 255;
				total = total + histogramIntensity[q]; //totalΪ�ܺͣ��ۼ�ֵ
			}
			//ƽ���������2��+�м�1��+�ұ�2���Ҷȣ���5���������ܺͳ���5�������0.5��������ֵ
			histogramIntensity[k] = (int)((float)total / 5.0 + 0.5);
		}
	}

	/* 3 ������ֵ */
	double sum = 0.0;
	int n = 0;
	//�����ܵ�ͼ��ĵ����������أ�Ϊ����ļ�����׼��
	for (int k = minIntensity; k <= maxIntensity; k++)
	{
		//x*f(x)�����أ�Ҳ����ÿ���Ҷȵ�ֵ�������������һ����Ϊ���ʣ���sumΪ���ܺ�
		sum += (double)k * (double)histogramIntensity[k];
		n += histogramIntensity[k]; //nΪͼ���ܵĵ�������һ��������ۻ�����
	}

	double otsu = -1.0;
	int n1 = 0;
	double csum = 0.0;
	for (int k = minIntensity; k <= maxIntensity; k++) //��ÿ���Ҷȣ���0��255������һ�ηָ�����䷽��sb
	{
		n1 += histogramIntensity[k]; //n1Ϊ�ڵ�ǰ��ֵ��ǰ��ͼ��ĵ���
		if (n1 == 0) { continue; } //û�зֳ�ǰ����
		int n2 = n - n1; //n2Ϊ����ͼ��ĵ���
		if (n2 == 0) { break; }//n2Ϊ0��ʾȫ�����Ǻ�ͼ����n1=0������ƣ�֮��ı���������ʹǰ���������ӣ����Դ�ʱ�����˳�ѭ��

		csum += (double)k * histogramIntensity[k]; //ǰ���ġ��Ҷȵ�ֵ*����������ܺ�
		double m1 = csum / n1; //m1Ϊǰ����ƽ���Ҷ�
		double m2 = (sum - csum) / n2; //m2Ϊ������ƽ���Ҷ�
		double sb = (double)n1 * (double)n2 * (m1 - m2) * (m1 - m2); //sbΪ��䷽��
		if (sb > otsu) //����������䷽�����ǰһ���������䷽��
		{
			otsu = sb; //otsuʼ��Ϊ�����䷽�otsu��
			thrIntensity = k; //ȡ�����䷽��ʱ��Ӧ�ĻҶȵ�k���������ֵ
		}
	}


	if (1 == 2)//method2
	{
		float tempVarValueTemp = 1.0f;
		for (int i = minIntensity; i <= maxIntensity; i++)
		{
			float w0 = 0, w1 = 0, u1 = 0, u0 = 0;
			for (int j = minIntensity; j <= i; j++)
			{
				w1 += histogramIntensity[j]; // �������ص�����
				u1 += j * histogramIntensity[j];// �����������ػҶ��ܺ�
			}
			if (w1 == 0)
			{
				continue;
			}

			u1 = u1 / w1; // ����ֵ��ƽ���Ҷ�
			w1 = w1 / pcCount; // �������ֵ�����ռ�ı���

			for (int k = i + 1; k <= maxIntensity; k++)
			{
				w0 += histogramIntensity[k]; // ǰ���������ص��ܺ�
				u0 += k * histogramIntensity[k]; // ǰ���������ػҶ��ܺ�
			}

			if (w0 == 0)
			{
				break;
			}

			u0 = u0 / w0;
			w0 = w0 / pcCount;

			float varValue = w0 * w1 * (u1 - u0)*(u1 - u0);
			if (tempVarValueTemp < varValue)
			{
				tempVarValueTemp = varValue;
				thrIntensity = i;
			}
		}
	}

	return thrIntensity;
}

/* �̱��� short marking: ��ȡ���ߵ� get traffic marking points */
std::vector<CCVector3> xjAlgorithm::xjGetTrafficMarkingPoints(ccPointCloud* cloud, const char *sf, 
	const uint &threshold, const bool &bCluster)
{
	std::vector<CCVector3> vMarkingPoints;

	ccScalarField* ccSF = static_cast<ccScalarField*>(cloud->getScalarField(cloud->getScalarFieldIndexByName(sf)));
	for (int i = 0; i < cloud->size(); i++)
	{
		if ((uint)ccSF->getValue(i) >= threshold)
		{
			CCVector3 p(cloud->getPoint(i)->x, cloud->getPoint(i)->y, cloud->getPoint(i)->z);
			vMarkingPoints.push_back(p);
		}
	}

	if (bCluster)
	{

	}

	return vMarkingPoints;
}


/* ͹�� convex hull */
//���� ������ƽ�����
double xjAlgorithm::TwoDistancePointAndPoint(const CCVector3 &p1, const CCVector3 &p2)
{
	double x1 = p1.x, y1 = p1.y;
	double x2 = p2.x, y2 = p2.y;

	double dis = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));

	return dis;
}
/*���� ���������Ĳ����p0�ǹ����㣩*/
double xjAlgorithm::xjGetCrossProduct(const CCVector3 & p1, const CCVector3 & p2, const CCVector3 & p0)
{
	return ((p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y));
}
/* ͹�� convex hull */
std::vector<CCVector3> xjAlgorithm::xjGetConvexHullByGrahamScan(std::vector<CCVector3> vPoint)
{
	std::vector<CCVector3> vConvexHullPoint;

	int i = 0, j = 0, k = 0;
	CCVector3 tmP = vPoint[0];

	//���� �ҵ�������ƫ��ĵ�
	for (i = 1; i < vPoint.size(); i++)
	{
		if ((vPoint[i].y < tmP.y) || ((vPoint[i].y == tmP.y) && (vPoint[i].x < tmP.x)))
		{
			tmP = vPoint[i];
			k = i;
		}
	}
	vPoint[k] = vPoint[0];
	vPoint[0] = tmP;

	//���� �����Ǵ�С���󣬾���ӽ���Զ 
	for (i = 1; i < vPoint.size(); i++)
	{
		k = i;
		for (j = i + 1; j < vPoint.size(); j++)
		{
			double cross = xjGetCrossProduct(vPoint[j], vPoint[k], vPoint[0]);
			double disance_j = TwoDistancePointAndPoint(vPoint[0], vPoint[j]);
			double disance_k = TwoDistancePointAndPoint(vPoint[0], vPoint[k]);
			if ((cross > 0) || ((cross == 0) && (disance_j < disance_k)))
			{
				k = j;//k���漫����С���Ǹ���,������ͬ����ԭ�����
			}
		}
		tmP = vPoint[i];
		vPoint[i] = vPoint[k];
		vPoint[k] = tmP;
	}

	//��ӵ�һ������͹����
	vConvexHullPoint.push_back(vPoint[0]);
	vConvexHullPoint.push_back(vPoint[1]);

	////����
	for (i = 2; i < vPoint.size(); ++i)
	{
		for (j = i + 1; j < vPoint.size(); ++j)
		{
			if (xjGetCrossProduct(vPoint[i], vPoint[j], vConvexHullPoint[vConvexHullPoint.size() - 1]) < 0)
			{
				i = j;
			}
		}
		vConvexHullPoint.push_back(vPoint[i]);
	}

	return vConvexHullPoint;
}
/* �������� */
double xjAlgorithm::ComputePolygonArea(const std::vector<CCVector3> &points)
{
	int point_num = points.size();
	if (point_num < 3)return 0.0;
	double s = 0;
	for (int i = 0; i < point_num; ++i)
		s += points[i].x * points[(i + 1) % point_num].y - points[i].y * points[(i + 1) % point_num].x;
	return fabs(s / 2.0);
}



/* ��С��Ӿ��� MBR */
//���� �㣨p0��  ���ߣ�p1��p2���ľ���
double xjAlgorithm::TwoDistancePointAndLine(const CCVector3 &p0, const CCVector3 &p1, const CCVector3 &p2)
{
	double dis12 = TwoDistancePointAndPoint(p1, p2);//�߶γ���
	double dis01 = TwoDistancePointAndPoint(p0, p1);//p1��p0�ľ���
	double dis02 = TwoDistancePointAndPoint(p0, p2);//p2��p0�ľ���
	double HalfC = (dis12 + dis01 + dis02) / 2;// ���ܳ�
	double s = sqrt(HalfC * (HalfC - dis12) * (HalfC - dis01) * (HalfC - dis02));//���׹�ʽ�����
	double xj2DisPL = 2 * s / dis12;// ���ص㵽�ߵľ��루���������������ʽ��ߣ�

	return xj2DisPL;
}
/* MBR */
/*���� ��С��Ӿ��ε�*/
/* xjListCH ͹���� */
std::vector<CCVector3> xjAlgorithm::xjMinimumBoundingRectangle(double &length, double &width, const std::vector<CCVector3> &vCHpoint)
{
	std::vector<CCVector3> xjMBRpoint(4);

	//��Ӿ���
	QList<CCVector3> xjListCH;
	QList<CCVector3> xjListCH2;
	int xjPcount = vCHpoint.size();
	for (int i = 0; i < vCHpoint.size(); i++)
	{
		xjListCH.append(vCHpoint.at(i));
		xjListCH2.append(vCHpoint.at(i));
	}
	qSort(xjListCH2.begin(), xjListCH2.end(), [](const CCVector3 &a, const CCVector3 &b) {return a.x < b.x; });
	double maxX = xjListCH2.at(xjPcount - 1).x;
	double minX = xjListCH2.at(0).x;

	qSort(xjListCH2.begin(), xjListCH2.end(), [](const CCVector3 &a, const CCVector3 &b) {return a.y < b.y; });
	double maxY = xjListCH2.at(xjPcount - 1).y;
	double minY = xjListCH2.at(0).y;

	qSort(xjListCH2.begin(), xjListCH2.end(), [](const CCVector3 &a, const CCVector3 &b) {return a.z < b.z; });
	double maxZ = xjListCH2.at(xjPcount - 1).z;
	double minZ = xjListCH2.at(0).z;

	//�����ж�
	double minArea = 99999999;
	xjListCH.push_back(xjListCH.at(0));
	for (int a = 0; a < xjListCH.size() - 1; a++)
	{
		CCVector3 p0 = xjListCH.at(a);
		CCVector3 p1 = xjListCH.at(a + 1);

		if ((p0.y == p1.y) || (p0.x == p1.x)) // ˮƽ��ֱ
		{
			double side1 = maxY - minY;
			double side0 = maxX - minX;
			double xjArea = side0 * side1;

			if (xjArea <= minArea)
			{
				length = std::max(side0, side1);
				width = std::min(side0, side1);
				minArea = xjArea;

				//��Ӿ����ĸ���
				CCVector3 pLB;
				pLB.x = minX;
				pLB.y = minY;
				pLB.z = maxZ;
				CCVector3 pRB;
				pRB.x = maxX;
				pRB.y = minY;
				pRB.z = maxZ;
				CCVector3 pRT;
				pRT.x = maxX;
				pRT.y = maxY;
				pRT.z = maxZ;
				CCVector3 pLT;
				pLT.x = minX;
				pLT.y = maxY;
				pLT.z = maxZ;

				xjMBRpoint.clear();
				xjMBRpoint.push_back(pLB);
				xjMBRpoint.push_back(pRB);
				xjMBRpoint.push_back(pRT);
				xjMBRpoint.push_back(pLT);
			}
		}
		else //��ˮƽ ����ֱ
		{
			double k1 = (p1.y - p0.y) / (p1.x - p0.x);
			double b1 = p0.y - k1 * p0.x;
			double side0 = -3;
			CCVector3 Pside0;
			for (int j = 0; j < xjListCH.size(); j++)
			{
				if ((j == a) || (j == (a + 1)))
					continue;

				CCVector3 p = xjListCH.at(j);
				double dis = abs(TwoDistancePointAndLine(p, p0, p1));
				if (dis >= side0)
				{
					side0 = dis;
					Pside0.x = p.x;
					Pside0.y = p.y;
					Pside0.z = p.z;
				}
			}
			double b11 = Pside0.y - k1 * Pside0.x;

			//��ֱ����
			double k2 = -1.0 / k1;
			double bb = p0.y - k2 * p0.x;
			double side1_positive = -3;
			CCVector3 Pside1_positive;
			double side1_negative = 9999999;
			CCVector3 Pside1_negative;
			for (int j = 0; j < xjListCH.size(); j++)
			{
				CCVector3 p = xjListCH.at(j);
				double dis = (k2*p.x - p.y + bb) / (sqrt(k2*k2 + 1));
				if ((dis >= 0) && (dis >= side1_positive))
				{
					side1_positive = dis;
					Pside1_positive.x = p.x;
					Pside1_positive.y = p.y;
					Pside1_positive.z = p.z;
				}
				if ((dis < 0) && (dis <= side1_negative))
				{
					side1_negative = dis;
					Pside1_negative.x = p.x;
					Pside1_negative.y = p.y;
					Pside1_negative.z = p.z;
				}
			}

			double b2 = Pside1_positive.y - k2 * Pside1_positive.x;
			double b22 = Pside1_negative.y - k2 * Pside1_negative.x;

			//������ܳ�
			double side1 = abs(side1_positive) + abs(side1_negative);
			double xjArea = side0 * side1;

			if (xjArea <= minArea)
			{
				length = std::max(side0, side1);
				width = std::min(side0, side1);
				minArea = xjArea;

				//��Ӿ����ĸ���
				CCVector3 br0;
				br0.x = (b1 - b22) / (k2 - k1);
				br0.y = k1 * br0.x + b1;
				br0.z = maxZ;
				CCVector3 br1;
				br1.x = (b11 - b22) / (k2 - k1);
				br1.y = k1 * br1.x + b11;
				br1.z = maxZ;
				CCVector3 br2;
				br2.x = (b2 - b11) / (k1 - k2);
				br2.y = k1 * br2.x + b11;
				br2.z = maxZ;
				CCVector3 br3;
				br3.x = (b2 - b1) / (k1 - k2);
				br3.y = k1 * br3.x + b1;
				br3.z = maxZ;

				xjMBRpoint.clear();
				xjMBRpoint.push_back(br0);
				xjMBRpoint.push_back(br1);
				xjMBRpoint.push_back(br2);
				xjMBRpoint.push_back(br3);
			}
		}
	}

	//MBR��ʾ��Ϣ
	QString MBRinfo = "chMBR: length = " + QString::number(length, 'f', 4) + ", ";
	MBRinfo += "width = " + QString::number(width, 'f', 4) + ", ";
	MBRinfo += "minimum area = " + QString::number(minArea) + ", ";
	MBRinfo += "circumference = " + QString::number((length + width) * 2);

	//���
	return xjMBRpoint;
}


/* ----- �����߲ ·�������ȡ -------- */
/* gridding voxel */
QMultiHash<int, xjPoint> xjAlgorithm::xjGridding(ccPointCloud *cloud, xjLasParameter &parameter)
{
	QMultiHash<int, xjPoint> mhGridding;
	mhGridding.reserve(cloud->size());

	CCVector3 bbMin, bbMax;
	cloud->getBoundingBox(bbMin, bbMax);
	parameter.maxX = bbMax.x;
	parameter.minX = bbMin.x;
	parameter.maxY = bbMax.y;
	parameter.minY = bbMin.y;
	parameter.maxZ = bbMax.z;
	parameter.minZ = bbMin.z;
	parameter.sumRow = (int)((bbMax.y - bbMin.y) / parameter.GSD) + 1;
	parameter.sumCol = (int)((bbMax.x - bbMin.x) / parameter.GSD) + 1;
	parameter.sumLay = 1;
	if (parameter.bVoxel)
	{
		parameter.sumLay = (int)((bbMax.z - bbMin.z) / parameter.GSD) + 1;
	}
	
	for (int i = 0; i < cloud->size(); i++)
	{
		xjPoint p;
		p.x = cloud->getPoint(i)->x;
		p.y = cloud->getPoint(i)->y;
		p.z = cloud->getPoint(i)->z;
		p.PID = i;

		p.RowNumber = (int)((p.y - bbMin.y) / parameter.GSD) + 1;
		p.ColNumber = (int)((p.x - bbMin.x) / parameter.GSD) + 1;
		p.LayNumber = 1;
		if (parameter.bVoxel)
		{
			p.LayNumber = (int)((p.z - bbMin.z) / parameter.GSD) + 1;
		}
		mhGridding.insert(((p.LayNumber - 1)*parameter.sumRow*parameter.sumCol + (p.RowNumber - 1)*parameter.sumCol + p.ColNumber), p);
	}

	return mhGridding;
}

/* denoising */
void xjAlgorithm::xjDenoising(QMultiHash<int, xjPoint> &mhGridding, xjLasParameter &parameter)
{
	QMultiHash<int, xjPoint>::iterator i;
	for (i = mhGridding.begin();i != mhGridding.end(); ++i)
	{
		if (mhGridding.values(i.key()).size() < parameter.desnoseCount)
			mhGridding.remove(i.key());
	}
}

/* collection of grid number */
QMultiHash<int, int> xjAlgorithm::xjGetGridNumber(const QMultiHash<int, xjPoint> &mhGridding, xjLasParameter &parameter)
{
	QMultiHash<int, int> mhGN;

	float thrZ = parameter.GSD* tan(3.1415926 * parameter.thrSlope / 180);

	QMultiHash<int, int> mhKey;
	for (QMultiHash<int, xjPoint>::const_iterator it = mhGridding.constBegin(); it != mhGridding.constEnd(); ++it)
	{
		int gn = it.key();
		if (mhKey.contains(gn)) { continue; }
		mhKey.insert(gn, gn);

		/* elevation statistical filter */
		QList<xjPoint> listP = mhGridding.values(it.key());
		if (listP.size() < parameter.desnoseCount) { continue; }
		qSort(listP.begin(), listP.end(), [](const xjPoint &a, const xjPoint &b) {return a.z < b.z; });
		//listPesf = xjElevationStatisticalFilter(listP, parameter);
		float deltaZ = listP.at(listP.size() - 1).z - listP.at(0).z;

		/* distinguish shape */
		double includedAngle = 0;
		int shape = xjEigenValueVectorShape(listP, includedAngle);

		/* result */
		if ((1 == 1)
			&& (deltaZ < thrZ)
			&& (shape ==2)
			)
		{
			mhGN.insert(gn,gn);
		}
	}

	return mhGN;
}

/* ����ֵ���������� eigenvalue and eigenvector */
int xjAlgorithm::xjEigenValueVectorShape(const QList<xjPoint> &listP, double &includedAngle)
{
	int shape = 1;


#pragma region average
	float meanX = 0.0f, meanY = 0.0f, meanZ = 0.0f;
	int count = listP.size();
	for (unsigned i = 0; i < count; i++)
	{
		meanX += listP.at(i).x;
		meanY += listP.at(i).y;
		meanZ += listP.at(i).z;
	}
	meanX /= count;
	meanY /= count;
	meanZ /= count;
#pragma endregion

	double mXX = 0.0;
	double mYY = 0.0;
	double mZZ = 0.0;
	double mXY = 0.0;
	double mXZ = 0.0;
	double mYZ = 0.0;
	for (int i = 0; i < count; ++i)
	{
		mXX += static_cast<double>(listP.at(i).x - meanX)*(listP.at(i).x - meanX);
		mYY += static_cast<double>(listP.at(i).y - meanY)*(listP.at(i).y - meanY);
		mZZ += static_cast<double>(listP.at(i).z - meanZ)*(listP.at(i).z - meanZ);
		mXY += static_cast<double>(listP.at(i).x - meanX)*(listP.at(i).y - meanY);
		mXZ += static_cast<double>(listP.at(i).x - meanX)*(listP.at(i).z - meanZ);
		mYZ += static_cast<double>(listP.at(i).y - meanY)*(listP.at(i).z - meanZ);
	}
	mXX /= count;
	mYY /= count;
	mZZ /= count;
	mXY /= count;
	mXZ /= count;
	mYZ /= count;

	/* Э������� */
	Matrix3d eMat;
	eMat(0, 0) = mXX; eMat(0, 1) = mXY; eMat(0, 2) = mXZ;
	eMat(1, 0) = mXY; eMat(1, 1) = mYY; eMat(1, 2) = mYZ;
	eMat(2, 0) = mXZ; eMat(2, 1) = mYZ; eMat(2, 2) = mZZ;

	/* ����ֵ���������� */
	Eigen::EigenSolver<Eigen::Matrix3d> xjMat(eMat);
	Matrix3d eValue = xjMat.pseudoEigenvalueMatrix();
	Matrix3d eVector = xjMat.pseudoEigenvectors();
	double v1 = eValue(0, 0);
	double v2 = eValue(1, 1);
	double v3 = eValue(2, 2);
	//eigenvalue
	QList<float> listEV;
	listEV << v1 << v2 << v3;
	qSort(listEV.begin(), listEV.end());//��С����
	v1 = listEV.at(2);
	v2 = listEV.at(1);
	v3 = listEV.at(0);
	//calculate
	double a1 = (std::sqrt(v1) - std::sqrt(v2)) / std::sqrt(v1);
	double a2 = (std::sqrt(v2) - std::sqrt(v3)) / std::sqrt(v1);
	double a3 = std::sqrt(v3 / v1);
	if ((std::max((std::max(a1, a2)), a3)) == a2) { shape = 2; }
	else if ((a3 >= a1) && (a3 >= a2)) { shape = 3; }


	/* normal */
	int minNumber = 0;
	if ((abs(v2) <= abs(v1)) && (abs(v2) <= abs(v3)))
	{
		minNumber = 1;
	}
	if ((abs(v3) <= abs(v1)) && (abs(v3) <= abs(v2)))
	{
		minNumber = 2;
	}
	double A = eVector(0, minNumber);
	double B = eVector(1, minNumber);
	double C = eVector(2, minNumber);
	double D = -(A*meanX + B * meanY + C * meanZ);
	if (C < 0)
	{
		A *= -1;
		B *= -1;
		C *= -1;
		D *= -1;
	}

	/* angle with Z-axis */
	double angle = (180.0 / 3.1415926)*acos(C / (sqrt(A*A + B * B + C * C)));
	includedAngle = abs(angle - 90);

	return shape;
}

/* cluster */
QList<QList<int>> xjAlgorithm::xjCluster(const QMultiHash<int, int> &mhGridNumber, xjLasParameter &parameter)
{
	QList<QList<int>> listCollection;

	QMultiHash<int, int> mhKey;
	mhKey.reserve(mhGridNumber.size());
	for (QMultiHash<int, int>::const_iterator ind = mhGridNumber.constBegin(); ind != mhGridNumber.constEnd(); ++ind)
	{
		int cgn = ind.key();
		if (mhKey.contains(cgn))
			continue;
		mhKey.insert(cgn, cgn);

		// region growth
		QList<int> list;
		RegionGrowth(list, cgn, parameter.sumCol, mhGridNumber, mhKey);

		if (list.size() >= parameter.clusterCount)
			listCollection.append(list);
	}

	qSort(listCollection.begin(), listCollection.end(), [](const QList<int> &a, const QList<int> &b) {return a.size() > b.size(); });

	return listCollection;
}
//interation
void xjAlgorithm::RegionGrowth(QList<int> &list, const int &cGridNumber, const int &sumCol, const QMultiHash<int, int> &mhGridNumber, QMultiHash<int, int> &mhKey)
{
	if (!list.contains(cGridNumber))
		list.append(cGridNumber);

	int row = 0, col = 0;
	GetRowAndCol(row, col, cGridNumber, sumCol);

	for (int r = row - 1; r < row + 2; r++)
	{
		for (int c = col - 1; c < col + 2; c++)
		{
			int gn = (r - 1)*sumCol + c;
			int neighborCount = xjGetBeighborCount(r, c, sumCol, mhGridNumber);
			if (
				(mhGridNumber.contains(gn)) 
				&& (!mhKey.contains(gn))
				&& (neighborCount>3)
				)
			{
				list.append(gn);
				mhKey.insert(gn, gn);

				RegionGrowth(list, gn, sumCol, mhGridNumber, mhKey);
			}
		}
	}
}
//�����������
int xjAlgorithm::xjGetBeighborCount(const int &row, const int &col, const int &sumCol, const QMultiHash<int, int> &mhGridNumber)
{
	int neighborCount = 0;
	for (int r = row - 1; r < row + 2; r++)
	{
		for (int c = col - 1; c < col + 2; c++)
		{
			int gn = (r - 1)*sumCol + c;
			if (mhGridNumber.contains(gn))
				neighborCount++;
		}
	}

	return neighborCount;
}





/* property of grid */
QMultiHash<int, xjGrid> xjAlgorithm::xjGridProperty(const QMultiHash<int, xjPoint> &mhGridding, xjLasParameter &parameter)
{
	QMultiHash<int, xjGrid> mhGridProperty;
	mhGridProperty.reserve(parameter.sumRow*parameter.sumCol*parameter.sumLay);

	for (QMultiHash<int, xjPoint>::const_iterator i = mhGridding.constBegin(); 
		i != mhGridding.constEnd(); ++i)
	{
		QList<xjPoint> listP = mhGridding.values(i.key());

		xjGrid g;
		g.pointCount = listP.size();
		qSort(listP.begin(), listP.end(), [](const xjPoint &a, const xjPoint &b) {return a.x < b.x; });
		g.maxX = listP.at(listP.size() - 1).x;
		g.minX = listP.at(0).x;
		qSort(listP.begin(), listP.end(), [](const xjPoint &a, const xjPoint &b) {return a.y < b.y; });
		g.maxY = listP.at(listP.size() - 1).y;
		g.minY = listP.at(0).y;
		qSort(listP.begin(), listP.end(), [](const xjPoint &a, const xjPoint &b) {return a.z < b.z; });
		g.maxZ = listP.at(listP.size() - 1).z;
		g.minZ = listP.at(0).z;
		g.deltaX = g.maxX - g.minX;
		g.deltaY = g.maxY - g.minY;
		g.deltaZ = g.maxZ - g.minZ;

		mhGridProperty.insert(i.key(), g);
	}
	
	return mhGridProperty;
}

/* �߳�ͳ�ƹ��� Elevation Statistical Filter */
QList<xjPoint> xjAlgorithm::xjElevationStatisticalFilter(const QList<xjPoint> &listP, const xjLasParameter &parameter)
{
	float deltaZ = listP.at(listP.size() - 1).z - listP.at(0).z;
	int sumLayer = int(deltaZ / parameter.GSD) + 1;

	QMultiHash<int, xjPoint> mhGroup;
	mhGroup.reserve(listP.size());
	int group = 0;
	for (int i = 0; i < listP.size(); i++)
	{
		group = int((listP.at(i).z - listP.at(0).z) / parameter.GSD) + 1;
		mhGroup.insert(group, listP.at(i));
	}

	for (int i = 1; i <= sumLayer; i++)
	{
		if (mhGroup.values(i).size() < parameter.desnoseCount)
			mhGroup.remove(i);
		else
			break;
	}
	for (int i = sumLayer; i > 0; i--)
	{
		if (mhGroup.values(i).size() < parameter.desnoseCount)
			mhGroup.remove(i);
		else
			break;
	}

	QList<xjPoint> listPesf;
	for (int i = 1; i <= sumLayer; i++)
	{
		if (mhGroup.contains(i))
		{
			for (int j = 0; j < mhGroup.values(i).size(); j++)
			{
				listPesf.append(mhGroup.values(i).at(j));
			}
		}
	}

	return listPesf;
}

//��ȡ�кź��к� get the row and column numbers based on the grid number
void xjAlgorithm::GetRowAndCol(int &row,int &col, const int &gridNumber, const int &sumCol)
{
	if (gridNumber <= sumCol)
	{
		row = 1;
		col = gridNumber;
	}
	else
	{
		int Quotient = (int)(gridNumber / sumCol);
		int Remainder = gridNumber % sumCol;//����
		if (Remainder == 0)
		{
			row = Quotient;
			col = sumCol;
		}
		else
		{
			row = Quotient + 1;
			col = Remainder;
		}
	}
}

/* mean Z */
float xjAlgorithm::xjMeanValue(const QList<xjPoint> &listP)
{
	float mean = 0.1f;
	for (int i = 0; i < listP.size(); i++)
	{
		mean += listP.at(i).z;
	}
	mean /= listP.size();

	return mean;
}

/* neighborhood point */
float xjAlgorithm::xjNeighbourhoodPoint(const QMultiHash<int, xjPoint> &mhGridding, xjLasParameter &parameter, const int &gn, const int &layer)
{
	float minZ=666666;

	int row = 0, col = 0;
	GetRowAndCol(row, col, gn, parameter.sumCol);
	for (int r = row-layer; r <=row+layer; r++)
	{
		for (int c = col - layer; c <= col + layer; c++)
		{
			int gn = (r - 1)*parameter.sumCol + c;
			if (mhGridding.contains(gn))
			{
				QList<xjPoint> listP = mhGridding.values(gn);
				qSort(listP.begin(), listP.end(), [](const xjPoint &a, const xjPoint &b) {return a.z < b.z; });
				if (listP.at(0).z<minZ)
				{
					minZ = listP.at(0).z;
				}
			}
		}
	}

	return minZ;
}


/* get row and col by grid number */
void xjAlgorithm::xjGetRowColByGN(const int &gridNumber, const int &sumCol, int &row, int &col)
{
	if (gridNumber <= sumCol)
	{
		row = 1;
		col = gridNumber;
	}
	else
	{
		int Quotient = (int)(gridNumber / sumCol);
		int Remainder = gridNumber % sumCol;//����
		if (Remainder == 0)
		{
			row = Quotient;
			col = sumCol;
		}
		else
		{
			row = Quotient + 1;
			col = Remainder;
		}
	}
}




/* create ccPointCloud */
void xjAlgorithm::xjCreatePointCloud(const QMultiHash<int, xjPoint> &mhGridding, xjLasParameter &parameter, ccPointCloud *newCloud)
{
	float thrZ = parameter.GSD* tan(3.1415926 * parameter.thrSlope / 180);
	int shape = 1;

	QMultiHash<int, int> mhKey;
	for (QMultiHash<int, xjPoint>::const_iterator it = mhGridding.constBegin(); it != mhGridding.constEnd(); ++it)
	{
		int gn = it.key();
		if (mhKey.contains(gn)) { continue; }
		mhKey.insert(gn, gn);

		/* row col number */
		int row = 0, col = 0;
		GetRowAndCol(row, col, gn, parameter.sumCol);
		float cx = parameter.minX + parameter.GSD*(col - 0.5);
		float cy = parameter.minY + parameter.GSD*(row - 0.5);

		/* elevation statistical filter */
		QList<xjPoint> listP = mhGridding.values(it.key());
		if (listP.size() < parameter.desnoseCount) { continue; }
		qSort(listP.begin(), listP.end(), [](const xjPoint &a, const xjPoint &b) {return a.z < b.z; });
		//listPesf = xjElevationStatisticalFilter(listP, parameter);
		float deltaZ = listP.at(listP.size() - 1).z - listP.at(0).z;


		///* ������͵� + thrZ */
		//QList<xjPoint> listPslope;
		//float minZ = xjNeighbourhoodPoint(mhGridding, parameter, gn, 2);
		//for (int i = 0; i < listP.size(); i++)
		//{
		//	if ((listP.at(i).z - minZ) < thrZ)
		//		listPslope.append(listP.at(i));
		//	else
		//		break;
		//}
		//if (listPslope.size() < parameter.desnoseCount) { continue; }


		/* shape: line plane sphere */
		//double includedAngle = 0;
		//int shape = xjEigenValueVectorShape(listP, includedAngle);


		/* result */
		if ((1 == 1)
			//&& (shape == 2)
			//&& (includedAngle<20)
			&& (deltaZ < thrZ)
			)
		{
			for (int j = 0; j < listP.size(); j++)
			{
				newCloud->addPoint(CCVector3(listP.at(j).x, listP.at(j).y, listP.at(j).z));
			}
		}
		//{
		//	newCloud->addPoint(CCVector3(cx, cy, listP.at(0).z));
		//}
	}
}


/* ---------- GDAL ---------- */
/* create polyline */
bool xjAlgorithm::xjCreatePolyline(ccPointCloud* cloud, const QString &shpPath)
{
	GDALAllRegister();
	OGRRegisterAll();
	const char *xjDriverName = "ESRI Shapefile";
	GDALDriver *xjDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(xjDriverName);
	if (xjDriver == NULL) { return false; }

	GDALDataset *xjDataset = xjDriver->Create(shpPath.toStdString().c_str(), 0, 0, 0, GDT_Unknown, NULL);
	if (xjDataset == NULL) { return false; }

	OGRLayer *xjLayer = xjDataset->CreateLayer("point_Line", NULL, wkbLineStringZM, NULL);
	if (xjLayer == NULL) { return false; }

	//���������ֶ�
	OGRFieldDefn fieldID("ID", OFTInteger);
	fieldID.SetWidth(32);
	xjLayer->CreateField(&fieldID);
	OGRFieldDefn fieldNAME("NAME", OFTString);
	fieldNAME.SetWidth(32);
	xjLayer->CreateField(&fieldNAME);
	OGRFieldDefn minX("LENGTH", OFTReal);
	minX.SetPrecision(16);
	xjLayer->CreateField(&minX);

	//����Ҫ��
	OGRFeature *xjFeature = OGRFeature::CreateFeature(xjLayer->GetLayerDefn());
	OGRLineString xjLine;
	for (int i = 0; i < cloud->size(); i++)
	{
		OGRPoint pt(cloud->getPoint(i)->x, cloud->getPoint(i)->y, cloud->getPoint(i)->z);
		xjLine.addPoint(&pt);

		///* test */
		//cloud->getScalarFieldIndexByName("Intensity");
	}
	OGRPoint pt0(cloud->getPoint(0)->x, cloud->getPoint(0)->y, cloud->getPoint(0)->z);
	xjLine.addPoint(&pt0);
	xjFeature->SetGeometry(&xjLine);

	//�ж�
	if (xjLayer->CreateFeature(xjFeature) != OGRERR_NONE) { return false; }
	OGRFeature::DestroyFeature(xjFeature);

	GDALClose(xjDataset);

	return true;
}

/* create polygon */
bool xjAlgorithm::xjCreatePolygon(ccPointCloud* cloud, const QString &shpPath)
{
	GDALAllRegister();
	OGRRegisterAll();
	const char *xjDriverName = "ESRI Shapefile";
	GDALDriver *xjDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(xjDriverName);
	if (xjDriver == NULL) { return false; }

	GDALDataset *xjDataset = xjDriver->Create(shpPath.toStdString().c_str(), 0, 0, 0, GDT_Unknown, NULL);
	if (xjDataset == NULL) { return false; }

	OGRLayer *xjLayer = xjDataset->CreateLayer("polygon1", NULL, wkbPolygonZM, NULL);
	if (xjLayer == NULL) { return false; }

	//���������ֶ�
	OGRFieldDefn fieldID("ID", OFTInteger);
	fieldID.SetWidth(32);
	xjLayer->CreateField(&fieldID);

	OGRFieldDefn fieldNAME("GLN", OFTString);
	fieldNAME.SetWidth(32);
	xjLayer->CreateField(&fieldNAME);

	OGRFieldDefn minX("minX", OFTReal);
	minX.SetPrecision(16);
	xjLayer->CreateField(&minX);

	OGRFieldDefn minY("minY", OFTReal);
	minY.SetPrecision(16);
	xjLayer->CreateField(&minY);

	OGRFieldDefn maxX("maxX", OFTReal);
	maxX.SetPrecision(16);
	xjLayer->CreateField(&maxX);

	OGRFieldDefn maxY("maxY", OFTReal);
	maxY.SetPrecision(16);
	xjLayer->CreateField(&maxY);

	int count = 0;
	QString gridNumber = "";
	for (int i = 0; i < cloud->size(); i += 4)
	{
		count++;

		//����Ҫ��
		OGRFeature *xjFeature = OGRFeature::CreateFeature(xjLayer->GetLayerDefn());
		//ʸ����Ҫ�صı߽��Ǳպϻ�
		OGRLinearRing xjRing;

		xjRing.addPoint(cloud->getPoint(i)->x, cloud->getPoint(i)->y, cloud->getPoint(i)->z);
		xjRing.addPoint(cloud->getPoint(i+1)->x, cloud->getPoint(i+1)->y, cloud->getPoint(i+1)->z);
		xjRing.addPoint(cloud->getPoint(i+2)->x, cloud->getPoint(i+2)->y, cloud->getPoint(i+2)->z);
		xjRing.addPoint(cloud->getPoint(i+3)->x, cloud->getPoint(i+3)->y, cloud->getPoint(i+3)->z);

		xjRing.closeRings();//��β���غ��γɱպϻ�

		 //ͼ�����Ҫ��
		OGRPolygon xjPolygon;
		xjPolygon.addRing(&xjRing);
		xjFeature->SetGeometry(&xjPolygon);

		//��������
		xjFeature->SetFID(i);
		xjFeature->SetField(0, i);
		int row = 0,col = 0;
		xjGetRowColByGN(count, 7, row, col);
		row += 442;
		gridNumber = "3103611700" + QString::number(row) + "0000" + QString::number(col);
		xjFeature->SetField(1, gridNumber.toStdString().c_str());
		xjFeature->SetField("minX", -cloud->getGlobalShift().x + cloud->getPoint(i)->x);
		xjFeature->SetField("minY", -cloud->getGlobalShift().y + cloud->getPoint(i)->y);
		xjFeature->SetField("maxX", -cloud->getGlobalShift().x + cloud->getPoint(i+2)->x);
		xjFeature->SetField("maxY", -cloud->getGlobalShift().y + cloud->getPoint(i+2)->y);

		//�ж�
		if (xjLayer->CreateFeature(xjFeature) != OGRERR_NONE) { return false; }
		OGRFeature::DestroyFeature(xjFeature);
	}

	GDALClose(xjDataset);

	return true;
}



/* ---------- eigen3 ---------- */

/* compute PCA(Principal Component Analysis) */
Eigen::Matrix4d xjAlgorithm::xjComputePCA(ccPointCloud *cloud)
{
#pragma region average
	float meanX = 0.0f, meanY = 0.0f, meanZ = 0.0f;
	unsigned count = cloud->size();
	for (unsigned i = 0; i < count; i++)
	{
		meanX += cloud->getPoint(i)->x;
		meanY += cloud->getPoint(i)->y;
		meanZ += cloud->getPoint(i)->z;
	}
	meanX /= count;
	meanY /= count;
	meanZ /= count;
#pragma endregion

#pragma region covariance mtrix
	double mXX = 0.0;
	double mYY = 0.0;
	double mZZ = 0.0;
	double mXY = 0.0;
	double mXZ = 0.0;
	double mYZ = 0.0;
	for (unsigned i = 0; i < count; ++i)
	{
		mXX += static_cast<double>(cloud->getPoint(i)->x - meanX)*(cloud->getPoint(i)->x - meanX);
		mYY += static_cast<double>(cloud->getPoint(i)->y - meanY)*(cloud->getPoint(i)->y - meanY);
		mZZ += static_cast<double>(cloud->getPoint(i)->z - meanZ)*(cloud->getPoint(i)->z - meanZ);
		mXY += static_cast<double>(cloud->getPoint(i)->x - meanX)*(cloud->getPoint(i)->y - meanY);
		mXZ += static_cast<double>(cloud->getPoint(i)->x - meanX)*(cloud->getPoint(i)->z - meanZ);
		mYZ += static_cast<double>(cloud->getPoint(i)->y - meanY)*(cloud->getPoint(i)->z - meanZ);
	}
	mXX /= count;
	mYY /= count;
	mZZ /= count;
	mXY /= count;
	mXZ /= count;
	mYZ /= count;

	/* ------------------ */
	Matrix3d eMat;
	eMat(0, 0) = mXX; eMat(0, 1) = mXY; eMat(0, 2) = mXZ;
	eMat(1, 0) = mXY; eMat(1, 1) = mYY; eMat(1, 2) = mYZ;
	eMat(2, 0) = mXZ; eMat(2, 1) = mYZ; eMat(2, 2) = mZZ;

	/* SVD�ֽ⣺A = U * S * V.T */
	JacobiSVD<Eigen::MatrixXd> svd(eMat, ComputeThinU | ComputeThinV);
	Matrix3d U = svd.matrixU(), V = svd.matrixV();
	Matrix3d S = U.inverse() * eMat * V.transpose().inverse();
	std::vector<double> eigenValues = { S(0, 0),S(1, 1),S(2, 2) };
	///*  */
	//Eigen::EigenSolver<Eigen::Matrix3d> xjMat(eMat);
	//Matrix3d eVector = xjMat.pseudoEigenvectors();
	//Matrix3d eValue = xjMat.pseudoEigenvalueMatrix();
	//std::vector<double> eigenValues = { eValue(0, 0),eValue(1, 1),eValue(2, 2) };

	/* result */
	Matrix3d sortEigenVectors;
	std::vector<double> sortEigenValues;
	xjSortEigenVectorByValues(U, eigenValues, sortEigenVectors, sortEigenValues);

	Matrix4d matPCA;
	matPCA(0, 0) = sortEigenVectors(0, 0); matPCA(0, 1) = sortEigenVectors(0, 1); matPCA(0, 2) = sortEigenVectors(0, 2); matPCA(0, 3) = 0;
	matPCA(1, 0) = sortEigenVectors(1, 0); matPCA(1, 1) = sortEigenVectors(1, 1); matPCA(1, 2) = sortEigenVectors(1, 2); matPCA(1, 3) = 0;
	matPCA(2, 0) = sortEigenVectors(2, 0); matPCA(2, 1) = sortEigenVectors(2, 1); matPCA(2, 2) = sortEigenVectors(2, 2); matPCA(2, 3) = 0;
	matPCA(3, 0) = sortEigenValues[0]; matPCA(3, 1) = sortEigenValues[1]; matPCA(3, 2) = sortEigenValues[2]; matPCA(3, 3) = 0;

	return matPCA;
#pragma endregion
}
/* sort eigen vector by eigen values */
void xjAlgorithm::xjSortEigenVectorByValues(const Matrix3d& eigenVectors, const std::vector<double>& eigenValues,
	Matrix3d& sortEigenVectors, std::vector<double>& sortEigenValues)
{
	if ((eigenValues[0]>= eigenValues[1])&& (eigenValues[0] >= eigenValues[2]))
	{
		sortEigenValues.push_back(eigenValues[0]);
		sortEigenVectors(0, 0) = eigenVectors(0, 0);
		sortEigenVectors(1, 0) = eigenVectors(1, 0);
		sortEigenVectors(2, 0) = eigenVectors(2, 0);
		if (eigenValues[1] >= eigenValues[2])
		{
			sortEigenValues.push_back(eigenValues[1]);
			sortEigenVectors(0, 1) = eigenVectors(0, 1);
			sortEigenVectors(1, 1) = eigenVectors(1, 1);
			sortEigenVectors(2, 1) = eigenVectors(2, 1);

			sortEigenValues.push_back(eigenValues[2]);
			sortEigenVectors(0, 2) = eigenVectors(0, 2);
			sortEigenVectors(1, 2) = eigenVectors(1, 2);
			sortEigenVectors(2, 2) = eigenVectors(2, 2);
		}
		else
		{
			sortEigenValues.push_back(eigenValues[2]);
			sortEigenVectors(0, 1) = eigenVectors(0, 2);
			sortEigenVectors(1, 1) = eigenVectors(1, 2);
			sortEigenVectors(2, 1) = eigenVectors(2, 2);

			sortEigenValues.push_back(eigenValues[1]);
			sortEigenVectors(0, 2) = eigenVectors(0, 1);
			sortEigenVectors(1, 2) = eigenVectors(1, 1);
			sortEigenVectors(2, 2) = eigenVectors(2, 1);
		}
	}
	else if ((eigenValues[1] >= eigenValues[0]) && (eigenValues[1] >= eigenValues[2]))
	{
		sortEigenValues.push_back(eigenValues[1]);
		sortEigenVectors(0, 0) = eigenVectors(0, 1);
		sortEigenVectors(1, 0) = eigenVectors(1, 1);
		sortEigenVectors(2, 0) = eigenVectors(2, 1);
		if (eigenValues[0] >= eigenValues[2])
		{
			sortEigenValues.push_back(eigenValues[0]);
			sortEigenVectors(0, 1) = eigenVectors(0, 0);
			sortEigenVectors(1, 1) = eigenVectors(1, 0);
			sortEigenVectors(2, 1) = eigenVectors(2, 0);

			sortEigenValues.push_back(eigenValues[2]);
			sortEigenVectors(0, 2) = eigenVectors(0, 2);
			sortEigenVectors(1, 2) = eigenVectors(1, 2);
			sortEigenVectors(2, 2) = eigenVectors(2, 2);
		}
		else
		{
			sortEigenValues.push_back(eigenValues[2]);
			sortEigenVectors(0, 1) = eigenVectors(0, 2);
			sortEigenVectors(1, 1) = eigenVectors(1, 2);
			sortEigenVectors(2, 1) = eigenVectors(2, 2);

			sortEigenValues.push_back(eigenValues[0]);
			sortEigenVectors(0, 2) = eigenVectors(0, 0);
			sortEigenVectors(1, 2) = eigenVectors(1, 0);
			sortEigenVectors(2, 2) = eigenVectors(2, 0);
		}
	}
	else if ((eigenValues[2] >= eigenValues[0]) && (eigenValues[2] >= eigenValues[1]))
	{
		sortEigenValues.push_back(eigenValues[2]);
		sortEigenVectors(0, 0) = eigenVectors(0, 2);
		sortEigenVectors(1, 0) = eigenVectors(1, 2);
		sortEigenVectors(2, 0) = eigenVectors(2, 2);
		if (eigenValues[0] >= eigenValues[1])
		{
			sortEigenValues.push_back(eigenValues[0]);
			sortEigenVectors(0, 1) = eigenVectors(0, 0);
			sortEigenVectors(1, 1) = eigenVectors(1, 0);
			sortEigenVectors(2, 1) = eigenVectors(2, 0);

			sortEigenValues.push_back(eigenValues[1]);
			sortEigenVectors(0, 2) = eigenVectors(0, 1);
			sortEigenVectors(1, 2) = eigenVectors(1, 1);
			sortEigenVectors(2, 2) = eigenVectors(2, 1);
		}
		else
		{
			sortEigenValues.push_back(eigenValues[1]);
			sortEigenVectors(0, 1) = eigenVectors(0, 1);
			sortEigenVectors(1, 1) = eigenVectors(1, 1);
			sortEigenVectors(2, 1) = eigenVectors(2, 1);

			sortEigenValues.push_back(eigenValues[0]);
			sortEigenVectors(0, 2) = eigenVectors(0, 0);
			sortEigenVectors(1, 2) = eigenVectors(1, 0);
			sortEigenVectors(2, 2) = eigenVectors(2, 0);
		}
	}
}

/* apply PCA */
ccPointCloud* xjAlgorithm::xjApplyPCA(ccPointCloud *cloud, const Matrix4d& matPCA)
{
	ccPointCloud *pcR = new ccPointCloud("PCA");
	if (!pcR->reserveThePointsTable(cloud->size()))
	{
		ccLog::Error("No enough memory!!!");
		delete pcR;
		pcR = nullptr;

		return nullptr;
	}

	double mX = 0, mY = 0, mZ = 0;
	for (int i = 0; i < cloud->size(); i++)
	{
		mX += cloud->getPoint(i)->x;
		mY += cloud->getPoint(i)->y;
		mZ += cloud->getPoint(i)->z;
	}
	mX /= cloud->size();
	mY /= cloud->size();
	mZ /= cloud->size();

	float x = 0, y = 0, z = 0;
	for (int i = 0; i < cloud->size(); i++)
	{
		x = matPCA(0, 0)*(cloud->getPoint(i)->x - mX) + matPCA(1, 0)*(cloud->getPoint(i)->y - mY) + matPCA(2, 0)*(cloud->getPoint(i)->z - mZ);
		y = matPCA(0, 1)*(cloud->getPoint(i)->x - mX) + matPCA(1, 1)*(cloud->getPoint(i)->y - mY) + matPCA(2, 1)*(cloud->getPoint(i)->z - mZ);
		z = matPCA(0, 2)*(cloud->getPoint(i)->x - mX) + matPCA(1, 2)*(cloud->getPoint(i)->y - mY) + matPCA(2, 2)*(cloud->getPoint(i)->z - mZ);
		CCVector3 p(x + mX, y + mY, z + mZ);
		pcR->addPoint(p);
	}
	pcR->invalidateBoundingBox();
	pcR->setPointSize(cloud->getPointSize() + 2);
	pcR->colorize(1, 0, 0);
	pcR->showColors(true);
	pcR->setGlobalScale(cloud->getGlobalScale());
	pcR->setGlobalShift(cloud->getGlobalShift());
	pcR->setDisplay(cloud->getDisplay());
	pcR->prepareDisplayForRefresh();
	pcR->refreshDisplay();
	if (cloud->getParent())
		cloud->getParent()->addChild(pcR);
	MainWindow::TheInstance()->addToDB(pcR);
}