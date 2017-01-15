#pragma once
#ifndef _GUARD_TYPEDEFINE_H
#define _GUARD_TYPEDEFINE_H
#include <vector>
#include <string>
#include <iostream>
#include "RTree.h"

using namespace std;
#define LATITUDE x
#define LONGTITUDE y

struct Point
{
	Point(){}
	bool operator == (const Point&b)
	{
		return x == b.x && y == b.y;
	}
	Point(double x_, double y_) :x(x_), y(y_){}
	double x;
	double y;
};

struct Point3
{
	double x;
	double y;
	double t;
};

struct Time
{
	Time() :day(0),hour(0),min(0),sec(0){}
	Time(int d, int h, int m, int s) :day(d), hour(h), min(m), sec(s){}
	Time(int h, int m, int s) :hour(h), min(m), sec(s){ day = 0; }
	void operator += (const Time & a)
	{
		hour += a.hour;
		min += a.min;
		sec += a.sec;
		if (sec >= 60)	{ sec -= 60; min++; }
		if (min >= 60)	{ min -= 60; hour++; }
		if (hour >= 24) { hour -= 24; day++; }
	}
	void operator -= (const Time & a)
	{
		hour -= a.hour;
		min -= a.min;
		sec -= a.sec;
		if (sec < 0)	{ sec += 60; min--; }
		if (min < 0)	{ min += 60; hour--; }
		if (hour<0) { hour += 24; day--; }
	}
	int day;
	int hour;
	int min;
	int sec;
};

struct Mappointdata
{
	Mappointdata(){}
	Mappointdata(double x_, double y_) :x(x_), y(y_){}
	double x;
	double y;
	int id;
	vector<Point> edge;
};
typedef Mappointdata* MAPDATA;

struct Maproaddata
{
	Point start;
	Point end;
};
typedef Maproaddata* ROADDATA;

struct TimePoint
{
	TimePoint(){}
	TimePoint(double x_, double y_, int hour_, int min_, int sec_) :x(x_), y(y_){ time.day = 0; time.hour = hour_; time.min = min_; time.sec = sec_; }
	double x;
	double y;
	Time time;
};

struct Point_info_withstr
{
	Point_info_withstr():ismainpoint(0){}
	int ismainpoint;
	double x; 
	double y; 
	double dis;
	char str[1000];
	char tag[100];
};


struct Taxiinoff
{
	int id;
	Point pt;
	Time time;
	char tag[100];
};


class Taxi
{
public:
	Taxi():totaldist(0),profit(0){};
	Taxi(int id_, int moneysum_ = 0):id(id_),moneysum(moneysum_),totaldist(0),profit(0){}
	int id;
	int moneysum;
	int posx;
	int posy;
	int notempty;
	double profit;
	double totaldist;

	void profitcalc();
	void write_profit_log(FILE* wp);
	void write_takeoff_log(FILE* wp);

	vector<TimePoint> startpoint;
	vector<TimePoint> endpoint;
	vector<double> dist;
};

struct Taxiquery
{
	int taxiid;
	Time time;
	Point onroadsrc;
	Point vsrca;
	Point vsrcb;
	Point onroaddst;
	Point vdsta;
	Point vdstb;
};

struct Featurepointdata
{
	Point a;
	Point b;
	double sec;
	union
	{
		double speed;
		double delay;
	};
};

typedef RTree<Maproaddata*, double, 2>  RTREE_WITH_ROAD;
typedef RTree<Mappointdata*, double, 2> RTREE_WITH_POINT;
typedef RTree<Featurepointdata*, double, 3> RTREE_WITH_FEA;
#endif
