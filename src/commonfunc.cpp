#include "commonfunc.h"
#include <cmath>
#include <vector>

using namespace std;
void strconvert(string a, char* buf, int n)
{
	_ASSERT(a.size() < n - 1);
	for (size_t i = 0; i < a.size(); ++i)
		buf[i] = a[i];
	buf[a.size()] = 0;
}

void getFiles(string path, vector<string>& files)
{
	long   hFile = 0;
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

double distcalc(double laa, double lab, double loa, double lob)
{
	if (laa == lab && loa == lob) return 0;
	double t = (loa - lob)*avgcos * 6371 * 2 * 3.1415926535 / 360;
	t *= t;
	double t2 = (laa - lab) * 6371 * 2 * 3.1415926535 / 360;
	t2 *= t2;
	return sqrt(t + t2);
}

void co_convert(double la, double lo, double &x, double &y)
{
	x = lo *avgcos * 6371 * 2 * 3.1415926535 / 360;
	y = la * 6371 * 2 * 3.1415926535 / 360;
}

double p2segdist(double x, double y, double x1, double y1, double x2, double y2)
{
	double t = p2linedist2(x, y, x1, y1, x2, y2);
	return sqrt(t);
}

double p2segdist2(double x, double y, double x1, double y1, double x2, double y2)
{
	double crossm = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
	if (crossm <= 0)
		return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	double dd = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	if (crossm >= dd)
		return sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2));
	double r = crossm / dd;
	double px = x1 + (x2 - x1) * r;
	double py = y1 + (y2 - y1) * r;
	return (x - px) * (x - px) + (py - y) * (py - y);
}

Point p2segshadow(double x, double y, double x1, double y1, double x2, double y2)
{
	Point t;
	double crossm = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
	if (crossm <= 0)
	{
		t.x = x1; t.y = y1;
		return t;
	}
	double dd = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	if (crossm >= dd)
	{
		t.x = x2;
		t.y = y2;
		return t;
	}
	double r = crossm / dd;
	double px = x1 + (x2 - x1) * r;
	double py = y1 + (y2 - y1) * r;
	t.x = px; t.y = py;
	return t;
}

double p2linedist2(double x, double y, double x1, double y1, double x2, double y2)
{
	double crossm = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
	double dd = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	double r = crossm / dd;
	double px = x1 + (x2 - x1) * r;
	double py = y1 + (y2 - y1) * r;
	return (x - px) * (x - px) + (py - y) * (py - y);
}

double p2linedist(double x, double y, double x1, double y1, double x2, double y2)
{
	return sqrt(p2linedist2(x,y,x1,y1,x2,y2));
}

double p2pdist2(double x1, double y1, double x2, double y2)
{
	return (x2 - x1)*(x2 - x1) + (y2 - y1) * (y2 - y1);
}
double p2pdist(double x1, double y1, double x2, double y2)
{
	return sqrt(p2pdist2(x1, y1, x2, y2));
}

Time operator - (const Time &a,const Time &b){
	Time e = a;
	e -= b;
	return e;
}

Time operator + (const Time &a, const Time &b)
{
	Time e = a;
	e += b;
	return e;
}

bool operator < (const Time& a, const Time& b)
{
	int hasha = a.day * 86400 + a.hour * 3600 + a.min * 60 + a.sec;
	int hashb = b.day * 86400 + b.hour * 3600 + b.min * 60 + b.sec;
	return hasha < hashb;
}

double earncalc(double dist)
{
	if (dist < 3)
		return 14;
	else
	{
		return 14 + 2.4*(dist - 3);
	}
}
double p2pdist(Point a, Point b)
{
	return sqrt(p2pdist2(a, b));
}

double p2pdist2(Point a, Point b)
{
	return p2pdist2(a.x, a.y, b.x, b.y);
}

double dim3dist(double x1, double y1, double t1, double x2, double y2, double t2)
{
	return sqrt(dim3dist2(x1,y1,t1,x2,y2,t2));
}

double dim3dist2(double x1, double y1, double t1, double x2, double y2, double t2)
{
	double tgap = abs(t2 - t1) > 43200 ? 86400 - abs(t2 - t1) : abs(t2 - t1);
	return (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + tgap * tgap;
}

double p2surfacedist(double x, double y, double t, Featurepointdata* a) //x,y,t(sec),a,b
{
	double segstartx = a->a.x;
	double segendx = a->b.x;
	double segstarty = a->a.y;
	double segendy = a->b.y;
	double segdist2 = p2segdist2(x, y, segstartx, segstarty, segendx, segendy);
	double gap = abs(t-  a->sec) > 43200 ? 86400 - abs(t - a->sec) : abs(t - a->sec);
	gap *= (1.0 / 3600);
	double dist = sqrt(segdist2 + gap * gap);
	return dist;
}