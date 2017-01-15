#pragma once
#ifndef _GUARD_ROADREADER_H
#define _GUARD_ROADREADER_H

#include <iostream>
#include "typedefine.h"
#include "RTree.h"
#include "RTree_addon.h"
#include "commonfunc.h"
#include <ctime>

#define EPS 0.00001

using namespace std;
class RoadreaderHandler
{
public:
	RoadreaderHandler(string s);
	~RoadreaderHandler(){ fclose(fp); }
	void savetree(string outpath);
	RTree<Mappointdata*, double, 2>& rtntree(){ return tree; }
	RTree<Maproaddata*, double, 2>& rtnrdtree(){ return rdtree; }
	void read_point_toTree();
	void read_road_toTree();
private:
	char path[1000];
	FILE* fp;
	RTree<Mappointdata*, double, 2> tree;
	RTree<Maproaddata*, double, 2> rdtree;
	int counter;
};

typedef Mappointdata* MAPDATA;

#endif
