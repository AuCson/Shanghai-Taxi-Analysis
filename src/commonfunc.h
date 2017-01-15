#pragma once
#ifndef _GUARD_COMMONFUNC_H
#define _GUARD_COMMONFUNC_H

#include<string>
#include <io.h>
#include <cmath>
#include <vector>
#include "typedefine.h"
using namespace std;

const double avgcos = cos(31.263319/180);
void strconvert(string a, char* buf, int n);
void getFiles(string path, vector<string>& files);
double distcalc(double laa, double lab, double loa, double lob);
void co_convert(double la, double lo, double &x, double &y);
double p2segdist(double x, double y, double x1, double y1, double x2, double y2);
double p2segdist2(double x, double y, double x1, double y1, double x2, double y2);
double p2linedist(double x, double y, double x1, double y1, double x2, double y2);
double p2linedist2(double x, double y, double x1, double y1, double x2, double y2);
double p2pdist2(double x1, double y1, double x2, double y2);
double p2pdist(double x1, double y1, double x2, double y2);
double p2pdist(Point a,Point b);
double p2pdist2(Point a,Point b);
Point p2segshadow(double x, double y, double x1, double y1, double x2, double y2);
Point NNpoint(double x, double y, RTREE_WITH_ROAD &tree, Point &neara, Point &nearb);
double dim3dist(double x1, double y1, double t1, double x2, double y2, double t2);
double dim3dist2(double x1, double y1, double t1, double x2, double y2, double t2);
double p2surfacedist(double x,double y,double time,Featurepointdata* a);

double earncalc(double dist);

Time operator - (const Time &a, const Time& b);
Time operator + (const Time &a, const Time &b);
bool operator < (const Time& a, const Time& b);


#endif

