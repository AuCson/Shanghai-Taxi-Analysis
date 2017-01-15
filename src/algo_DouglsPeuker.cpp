#include "Taxidatareader.h"
#include <queue>
#include <set>
#include <vector>
#include <algorithm>
#include "commonfunc.h"
using namespace std;
#define LATITUDE x
#define LONGTITUDE y
#define LIMIT 0.03

/*
	algo_DouglasPeuker.cpp
	
	������˹-�տ��㷨��һ�ָ�Чȴ���۲������ӵĹ켣���㷨������˼���ǽ���·��
	�нǽϴ�����ֱ�������С��ɾȥ�������˼򻯹켣���ݣ������ٹ켣����Ư�Ƶ���

	#define limit xxx ������С����ƫ�ԽСɾ���ĵ�ԽС��
	�ο�����α���������ɡ�
	�㷨�������Documentation��

*/

void TaxireaderHandler::Douglas_Peuker_simplify()
{
	for (size_t i = 0; i < filenames.size(); ++i)
	{
		char buf[1000];
		strcpy(buf, filenames[i].c_str());
		FILE* fp = fopen(buf, "r");
		Douglas_Peuker_simplify(fp,filenames[i],LIMIT);
		fclose(fp);
		cout << i + 1 << "complete" << endl;
	}
}

void TaxireaderHandler::Douglas_Peuker_simplify(FILE* fp,string filename,double limit)
{
	vector<Point_info_withstr> ptvec;
	vector<int> remain;
	Point_info_withstr t;
	char header[1000];

	char buf[100];
	char post[7] = "_M.csv";
	for (int i = 0; i < 7; ++i)
		buf[filename.size() - 4 + i] = post[i];

	int cnt = 0,previsempty = 0;
	fgets(header, 1000, fp);
	while (fgets(t.str, 1000, fp))
	{
		sscanf(t.str, "%*d,%*d-%*d-%*d %*d:%*d:%*d,,%lf,%lf,%*d,%*c,%*d,%*d,%*d,%*lf,%*lf,%*d,%s",
			&t.LONGTITUDE,&t.LATITUDE,t.tag);
		ptvec.push_back(t);
	}
	for (size_t i = 0; i < ptvec.size(); ++i)
	{
		if (ptvec[i].tag[10]=='I'||ptvec[i].tag[10]=='O')
			ptvec[i].ismainpoint = 1;
	}
	remain.resize(ptvec.size());
	Douglas_Peuker_simplify(0, ptvec.size() - 1, ptvec, remain,limit);

	for (int i = 0; i < filename.size() - 4; ++i)
	{
		buf[i] = filename[i];
	}
	for (int i = 0; i < 7; ++i)
		buf[filename.size() - 4 + i] = post[i];
	FILE* wfp = fopen(buf, "w");
	fprintf(wfp, "%s", header);
	for (size_t i = 0; i < remain.size(); ++i)
	{
		if (remain[i] || ptvec[i].ismainpoint )
			fprintf(wfp, "%s", ptvec[i].str);
	}
	fclose(wfp);
	
}

void TaxireaderHandler::Douglas_Peuker_simplify(int begin, int end, vector<Point_info_withstr>& pts, vector<int> &remain,double limit)
{
	if (end - begin <= 2)
	{
		remain[begin] = remain[end] = 1;
		return;
	}
	double beginx, beginy, endx, endy;
	co_convert(pts[begin].LATITUDE, pts[begin].LONGTITUDE, beginx, beginy);
	co_convert(pts[end].LATITUDE, pts[end].LONGTITUDE, endx, endy);
	double max = -1;
	int maxindex = -1;

	//ѡ�����߶���Զ�ĵ�
	for (int i = begin + 1; i <= end - 1; ++i)
	{
		double gx, gy;
		co_convert(pts[i].LATITUDE, pts[i].LONGTITUDE, gx, gy);
		pts[i].dis = p2segdist(gx, gy, beginx, beginy, endx, endy);
		if (pts[i].dis > max)
		{
			max = pts[i].dis;
			maxindex = i;
		}
	}
	_ASSERT(max >= 0 && maxindex >= 0);
	if (max >= limit)
	{
		remain[maxindex] = 1;
		Douglas_Peuker_simplify(begin, maxindex, pts, remain, limit);
		Douglas_Peuker_simplify(maxindex, end, pts, remain, limit);
	}
	else
	{
		remain[begin] = remain[end] = 1;
	}
	return;
}