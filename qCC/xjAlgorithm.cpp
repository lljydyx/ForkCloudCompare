#include "xjAlgorithm.h"



/* �̱��� short marking: view part result: ccPointCloud*/
void xjAlgorithm::ViewPartResult(ccPointCloud *cloud, const std::vector<CCVector3> &vPoint, 
	const QString &resultName, const int &pointSize, const float &r, const float &g, const float &b)
{
	ccPointCloud *pcPR = new ccPointCloud(resultName);
	if (!pcPR->reserveThePointsTable(vPoint.size()))
	{
		ccLog::Error("No enough memory!!!");
		delete pcPR;
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


/* ------------- */
/* gridding */
QMultiHash<int, xjPoint> xjAlgorithm::xjGridding(ccPointCloud *cloud, xjLasParameter &parameter)
{
	QMultiHash<int, xjPoint> mhGrid;
	mhGrid.reserve(cloud->size());

	CCVector3 bbMin, bbMax;
	cloud->getBoundingBox(bbMin, bbMax);
	parameter.sumRow = (int)((bbMax.y - bbMin.y) / parameter.GSD) + 1;
	parameter.sumCol = (int)((bbMax.x - bbMin.x) / parameter.GSD) + 1;
	parameter.sumLay = 1;
	//parameter.sumLay = (int)((bbMax.z - bbMin.z) / parameter.GSD) + 1;
	for (int i = 0; i < cloud->size(); i++)
	{
		xjPoint p;
		p.x = cloud->getPoint(i)->x;
		p.y = cloud->getPoint(i)->y;
		p.z = cloud->getPoint(i)->z;

		p.RowNumber = (int)((p.y - bbMin.y) / parameter.GSD) + 1;
		p.ColNumber = (int)((p.x - bbMin.x) / parameter.GSD) + 1;
		mhGrid.insert(((p.RowNumber - 1)*parameter.sumCol + p.ColNumber), p);
	}

	return mhGrid;
}

/* property of grid */
QMultiHash<int, xjGrid> xjAlgorithm::xjGridProperty(const QMultiHash<int, xjPoint> &mhGrid, xjLasParameter &parameter)
{
	QMultiHash<int, xjGrid> mhGridProperty;
	mhGridProperty.reserve(parameter.sumRow*parameter.sumCol*parameter.sumLay);

	for (QMultiHash<int, xjPoint>::const_iterator i = mhGrid.constBegin(); 
		i != mhGrid.constEnd(); ++i)
	{
		QList<xjPoint> listP = mhGrid.values(i.key);

		xjGrid g;
		g.pointCount = listP.size();
		xjPoint maxP, minP;
		qSort(listP.begin(), listP.end(), [](const xjPoint &a, const xjPoint &b) {return a.x < b.x; });
		maxP.x = listP.at(listP.size() - 1).x;
		minP.x = listP.at(0).x;
		qSort(listP.begin(), listP.end(), [](const xjPoint &a, const xjPoint &b) {return a.y < b.y; });
		maxP.y = listP.at(listP.size() - 1).y;
		minP.y = listP.at(0).y;
		qSort(listP.begin(), listP.end(), [](const xjPoint &a, const xjPoint &b) {return a.z < b.z; });
		maxP.z = listP.at(listP.size() - 1).z;
		minP.z = listP.at(0).z;
		g.maxP = maxP;
		g.minP = minP;
		g.deltaX = maxP.x - minP.x;
		g.deltaY = maxP.y - minP.y;
		g.deltaZ = maxP.z - minP.z;

		mhGridProperty.insert(i.key, g);
	}
	
	return mhGridProperty;
}

