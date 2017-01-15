#pragma once
#ifndef _GUARD_TAXIDATAREADER_H
#define _GUARD_TAXIDATAREADER_H

#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include "typedefine.h"
#include "commonfunc.h"
using namespace std;

class TaxireaderHandler
{
public:
	TaxireaderHandler(){}
	TaxireaderHandler(string dir)
	{
		getFiles(dir, filenames);
		counter = 0;
	}
	~TaxireaderHandler(){}
	void read(int profitcalc = 0);//for profit calc
	void Douglas_Peuker_simplify();
	void get_nearest_roadpts(RTREE_WITH_ROAD &tree);
	void write_take_log();
	void write_profit();
	
	void statistic(RTREE_WITH_ROAD &treerd);

private:
	vector<string> filenames;
	vector<Taxi*> Taxis;
	Taxi* readTaxi(FILE* fp,int profitcalc = 0);
	void Douglas_Peuker_simplify(FILE* fp,string filename, double limit);
	void Douglas_Peuker_simplify(int begin, int end, vector<Point_info_withstr> &pts, vector<int> &remain,double limit);
	void sort_and_split();
	void statistic_single(FILE* fp, RTREE_WITH_ROAD &treerd, FILE* log);
	Point get_nearest_roadpt(double x,double y,RTREE_WITH_ROAD &tree,Point &neara,Point &nearb);
	int counter;
	FILE* profitlog;
	FILE* take_offlog;
};



#endif
#pragma once