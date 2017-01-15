/*
#ifndef __GUARD__CSVREADER_H
#define __GUARD__CSVREADER_H

#include <iostream>
#include <fstream>
#include <string>
#include "RTree.h"
#include "commonfunc.h"
#include "RTree_Addon.h"


#include<vector>
using std::vector;

#define EPS 0.00001

typedef class Point_
{
public:
	Point_();
	Point_(double tx, double ty) :x(tx), y(ty){}
	double x;
	double y;
} Point;
typedef class Mappointdata_
{
public:
	Mappointdata_();
	Mappointdata_(double tx, double ty) :x(tx), y(ty){}
	double x;
	double y;
	vector<Point> edge;
} Mappointdata;

class RoadreaderHandler
{
public:
	RoadreaderHandler(istream& in);
	~RoadreaderHandler(){f.close();}
	fstream& rtnfile(){ return f; }
	void savetree(string outpath);
	RTree<Mappointdata*, double, 2>* rtntree(){ return &tree; }
	void readtoTree();
private:
	string path;
	fstream f;
	RTree<Mappointdata*, double, 2> tree;
};

typedef Mappointdata* MAPDATA;

bool _cdecl data_store_callback(MAPDATA data, void *arg);
void Insert_adj(RTree<Mappointdata*, double, 2> tree, const double a_min[2], const double a_max[2], Mappointdata* a_dataId, double eps);

#endif
#pragma once
*/