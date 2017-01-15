#include "RTree.h"
#include "RTree_Addon.h"
#include "typedefine.h"
#include "commonfunc.h"
#include <vector>
#include <list>
using namespace std;

class Predict_Base
{
public:
	Predict_Base(RTREE_WITH_POINT &ptree_,RTREE_WITH_ROAD& rdtree_) :ptree(ptree_), rdtree(rdtree_)
	{
		cout << "Velocity Staticstics file path..." << endl;
		scanf("%s", datasetdir);
		parameterK = 1;
		parameter =1;
	}
	void loaddataset();
	void iterforparameter();
	double predict(Point startpt, Point endpt,int hour,int min,int sec);
	void KNNpoints(double x,double y,double t,int k,vector<Featurepointdata*> &rtn,vector<double>& losses,double predictedmax);
	void KNNpointspt(double x, double y, double t, int k, vector<Featurepointdata*> &rtn, vector<double>& losses);
private:
	char datasetdir[1000];
	char delaydsdir[1000];
	Mappointdata* getpointdata(double x, double y);
	double Nfunc(double parameter, double loss);
	double Astar_time(Point startpos, Point endpos, double starttime,list<Point>& path);
	double Predict_road_time(Point a,Point b,double time);
	double predict_delay(Point a, double time);
	double parameter;
	double parameterK;
	RTREE_WITH_FEA ftree;
	RTREE_WITH_POINT &ptree;
	RTREE_WITH_FEA delaytree;
	RTREE_WITH_ROAD &rdtree;
};