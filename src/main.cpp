#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <string>
#include "Roadreader.h"
#include "Taxidatareader.h"
#include "AnalysisBase.h"
#include "Travel_time_predict.h"
#include <vector>
using namespace std;

#define EPS 0.00001

int main()
{
	RoadreaderHandler *rh = NULL;
	RoadreaderHandler *reh = NULL;
	TaxireaderHandler *th = NULL;
	Analysis_Base *ana_park = NULL;
	Predict_Base *predbase = NULL;
	
	string cmd,cmd2,cmd3;
	string path;

	while (1)
	{
		cout << "-STAT-" << endl << endl;

		if (rh != NULL) cout << "RoadData_AS_POINT Loaded" << endl;
		if (reh != NULL) cout << "RoadData_AS_ROAD Loaded" << endl;
		if (th != NULL) cout << "TaxiData(for calc) Loaded" << endl;
		if (ana_park != NULL) cout << "Analysis Class (park) Loaded" << endl;
		cin >> cmd;

		if (cmd == "LOAD")
		{
			cout << "[DATATYPE] [PATH]" << endl;
			cin >> cmd2;
			cin >> path;
			if (cmd2[0] == 'R')
			{
				cout << "[READTYPE]" << endl;
				cin >> cmd3;
				if (cmd3[0] == 'P'){
					rh = new RoadreaderHandler(path);
					rh->read_point_toTree();
				}
				else if (cmd3[0] == 'R')
				{
					reh = new RoadreaderHandler(path);
					reh->read_road_toTree();
				}

			}
			else if (cmd2[0] == 'T')
			{
				th = new TaxireaderHandler(path);
			}
		}
		else if (th != NULL && cmd == "ANATAXI")
		{
			th->write_profit();
			th->write_take_log();
		}
		else if (th != NULL && cmd == "SIMPLIFY")
		{
			th->Douglas_Peuker_simplify();
		}
		else if (th != NULL && reh!=NULL && cmd == "STATISTIC")
		{
			RTREE_WITH_ROAD& tree = reh->rtnrdtree();
			th->statistic(tree);
		}
		else if (reh != NULL && cmd == "MATCH")
		{
			RTREE_WITH_ROAD& tree = reh->rtnrdtree();
			th->get_nearest_roadpts(tree);
		}
		else if (reh != NULL && rh != NULL && cmd == "ANAPARK")
		{
			RTREE_WITH_ROAD& treerd = reh->rtnrdtree();
			RTREE_WITH_POINT& treept = rh->rtntree();
			ana_park = new Analysis_Base(treept, treerd);
			ana_park->analysis();
		}
		else if (reh != NULL && rh != NULL && cmd == "ANAMOVE")
		{
			RTREE_WITH_ROAD& treerd = reh->rtnrdtree();
			RTREE_WITH_POINT& treept = rh->rtntree();
			ana_park = new Analysis_Base(treept, treerd);
			ana_park->analysis_mov();
		}
		else if (reh != NULL && rh != NULL && cmd == "PREDICT")
		{
			RTREE_WITH_ROAD& treerd = reh->rtnrdtree();
			RTREE_WITH_POINT& treept = rh->rtntree();
			predbase = new Predict_Base(treept, treerd);
			predbase->loaddataset();
			while (1){
				cout << "<Start<x or longtitude>><End<y or latitude>><Time>" << endl;
				Point pta, ptb;
				int hour, min, sec;
				cin >> pta.x >> pta.y >> ptb.x >> ptb.y >> hour >> min >> sec;
				Point temp1 = pta, temp2 = ptb;
				if (temp1.x < 1000)
				{
					co_convert(temp1.x, temp1.y, pta.x, pta.y);
					co_convert(temp2.x, temp2.y, ptb.x, ptb.y);
				}
				predbase->predict(pta, ptb, hour, min, sec);
			}
		}
	}

}