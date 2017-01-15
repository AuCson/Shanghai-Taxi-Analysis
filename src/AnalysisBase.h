#pragma once
#ifndef _GUARD_ANA_PARK_H
#define _GUARD_ANA_PARK_H
#include "typedefine.h"
#include <vector>
#include <string>
#include <list>
#include "RTree.h"
#include "commonfunc.h"

using namespace std;

class Analysis_Base
{
public:
	struct Anainfo
	{
		int id;
		double profit[4];
	};
	Analysis_Base(RTREE_WITH_POINT& roaddata_, RTREE_WITH_ROAD &roaddatar) :roadtree(roaddata_), roadtreer(roaddatar)
	{
		//Required dataset : RoadNetwork as Vertex, RoadNetwork as Edge, Taxi5883, Takeinoff
		cout << "Taxi files..." << endl;
		string path;
		cin >> path;
		getFiles(path, files);
	}
	
	void analysis();
	void analysis_mov();

private:
	
	void addqtotree(Taxiquery *q);
	double Astar(double sx, double sy, double ex, double ey,list<Point> &path,int enable);
	void Analysis_Base::removequery(Taxiquery* q);
	Anainfo ana_single(Point startpt, Point startroadpta, Point startptroadb);
	Anainfo ana_single_m(Point startpt, Point startroadpta, Point startptroadb);
	Mappointdata* insertedge(Point onroad,Point va,Point vb);
	void read_query(string path);
	Mappointdata* getpointdata(double x, double y);
	void Analysis_Base::removeedge(double rdx, double rdy,Mappointdata* p);
	void getbestpt(Point sa, Point sb, Point ea, Point eb, Point &bests, Point &beste);
	vector<string> files;
	vector<Taxiquery*> query;
	vector<Anainfo> infos;
	Time nowtime;
	RTREE_WITH_POINT& roadtree;
	RTREE_WITH_ROAD& roadtreer;
	RTree<Taxiquery*, double, 2> querytree;
	
	Taxiquery* getnearest(Point startp);
	
};


#endif