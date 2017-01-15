#include "AnalysisBase.h"
#include "Travel_time_predict.h"
#include <queue>
#include <set>
#include <map>
#include <hash_map>
#include <unordered_map>
#include "RTree_Addon.h"
#include <list>
#include <cmath>

/*
	algo_Astar.cpp

	A star算法，参考伪代码教程独立完成.
	主要分为两个模块：
	1) A*常规距离最短路查询
	2) A*时间最优路快速启发式（可能极少量近似）查询

*/
#define EPS 0.00001
using namespace std;
#define KK 25

typedef RTREE_WITH_POINT::Node RNODE;

struct Vertexdata
{
	Mappointdata* mdata;
	double f;
};

class compare{
public:
	bool operator ()(Vertexdata a, Vertexdata b)
	{
		return a.f < b.f;
	}
};

Mappointdata* Analysis_Base::getpointdata(double x, double y)
{
	vector<MAPDATA*> container;
	double tmin[2];
	double tmax[2];
	tmin[0] = x - EPS; tmin[1] = y - EPS;
	tmax[0] = x + EPS; tmax[1] = y + EPS;
	int found = roadtree.Search(tmin, tmax, data_store_callback, &container);
	
	//ASSERT(found == 1);
	return *container[0];
}

bool same(double ax,double ay, double bx,double by)
{
	return abs(ax - bx) <= EPS && abs(ay - by) <= EPS;
}

double Analysis_Base::Astar(double sx, double sy, double ex, double ey,list<Point> &path,int enablestore)
{
	//cout << "begin" << endl;
	path.clear();
	int cnt = 0;
	int found = 0;
	set<Vertexdata, compare> open;

	unordered_map<Mappointdata*, Mappointdata*> come_from;
	unordered_map<Mappointdata*, double> cost_so_far;

	Mappointdata *me =  getpointdata(sx, sy);
	Vertexdata head;
	head.mdata = me;
	head.f = p2pdist(me->x, me->y, ex, ey);
	open.insert(head);

	come_from[me] = me;
	cost_so_far[me] = 0;
	Mappointdata *p;
	while (!open.empty())
	{
		p = open.begin()->mdata;
		double p_g = cost_so_far[p];
		open.erase(open.begin());

		if (same(p->x, p->y, ex, ey))
		{
			found = 1;
			break;
		}


		for (size_t i = 0; i < p->edge.size(); ++i)
		{
			cnt++;
			if (cnt % 20000 == 0) cout << cnt <<"Visited"<< endl;
			Mappointdata* c = getpointdata(p->edge[i].x, p->edge[i].y);
			double newcost = p_g + p2pdist(p->x, p->y, c->x, c->y);
			if (!cost_so_far.count(c))
			{
				cost_so_far[c] = newcost;
				double f = newcost + p2pdist(c->x, c->y, ex, ey);
				Vertexdata t;
				t.f = f;
				t.mdata =c;
				open.insert(t);
				come_from[c] = p;
			}
			else if (newcost < cost_so_far[c])
			{
				double f = cost_so_far[c] + p2pdist(c->x, c->y, ex, ey);
				Vertexdata tofind;
				tofind.f = f;
				tofind.mdata = c;
				auto it = open.find(tofind);
				_ASSERT(it != open.end());
				open.erase(it);

				tofind.f = newcost + p2pdist(c->x, c->y, ex, ey);
				open.insert(tofind);
				cost_so_far[c] = newcost;
				come_from[c] = p;
			}
		}

	}
	//cout << "Time:" << clock() - t1 << endl;
	double ans = cost_so_far[p];

	if (enablestore)
	{
		Mappointdata *pr = p;
		while (pr != come_from[pr])
		{ 
			Point t(pr->x, pr->y);
			path.push_front(t);
			pr = come_from[pr];
		}
	}
#ifdef PPATH
	FILE* log;
	log = fopen("path.txt", "w");
	for (auto i = path.begin(); i != path.end(); ++i)
		fprintf(log, "%lf,%lf\n", i->x, i->y);
	fclose(log);
#endif
	
	if (!found)
		return -1; //地图有不连通块，搜索失败！详情请见Documentaion.
	return ans;
}

double h_time(double x, double y, double ex, double ey)
{
	return p2pdist(x,y,ex,ey)/20.0;

}

Mappointdata* Predict_Base::getpointdata(double x, double y)
{
	vector<MAPDATA*> container;
	double tmin[2];
	double tmax[2];
	tmin[0] = x - EPS; tmin[1] = y - EPS;
	tmax[0] = x + EPS; tmax[1] = y + EPS;
	int found = ptree.Search(tmin, tmax, data_store_callback, &container);
//	_ASSERT(found >= 1);
	return *container[0];
}

/*dual buffer optimization

vector<Featurepointdata*> knn_hist;
int enable_cut;

little effect, not used.
*/
double Predict_Base::Predict_road_time(Point a, Point b, double time)//time unit:hour
{
	vector<Featurepointdata*> knna;
	vector<double> lossa;
	time *= 3600; // time unit is now sec
	double max_possible;
	max_possible = (double)(1 << 20);

	KNNpoints(a.x, a.y, time, KK, knna, lossa, max_possible);

	vector<Featurepointdata*> knnb;
	vector<double> lossb;

	KNNpoints(b.x, b.y, time, KK, knnb,lossb,max_possible);
	double predicted_spdA = 0;
	double predicted_spdB = 0;
	double coeffsuma = 0, coeffsumb = 0;
	for (int i = 0; i < KK; ++i)
	{
		double coeffa = Nfunc(parameter, lossa[i]);
		double coeffb = Nfunc(parameter, lossb[i]);
		coeffsuma += coeffa;
		coeffsumb += coeffb;
		predicted_spdA += coeffa * knna[i]->speed;
		predicted_spdB += coeffb * knnb[i]->speed;
	}
	predicted_spdA /= coeffsuma;
	predicted_spdB /= coeffsumb;

	double predictedspd = (predicted_spdA + predicted_spdB) / 2;
	return p2pdist(a, b) / predictedspd;
	
}

double Predict_Base::Astar_time(Point startpt, Point endpt, double starttime, list<Point>& path)
{
	path.clear();
	set<Vertexdata, compare> open;

	unordered_map<Mappointdata*, Mappointdata*> come_from;
	unordered_map<Mappointdata*, double> cost_so_far;


	int cnt = 0;
	Mappointdata *me = getpointdata(startpt.x,startpt.y);
	Vertexdata head;
	head.mdata = me;
	head.f = h_time(startpt.x, startpt.y, endpt.x, endpt.y);
	open.insert(head);

	come_from[me] = me;
	cost_so_far[me] = 0;
	Mappointdata *p;
	int t1 = clock();
	while (!open.empty())
	{
		p = open.begin()->mdata;
		double p_g = cost_so_far[p];
		open.erase(open.begin());
		if (same(p->x, p->y, endpt.x, endpt.y))
			break;

		for (size_t i = 0; i < p->edge.size(); ++i)
		{
			Mappointdata* c = getpointdata(p->edge[i].x, p->edge[i].y);
			cnt++;
			Point tc(c->x, c->y);
			Point tp(p->x, p->y);

			double predtime = Predict_road_time(tc, tp, starttime + cost_so_far[p]);
			double newcost = p_g + predtime;
			if (!cost_so_far.count(c))
			{
				cost_so_far[c] = newcost;
				double f = newcost + h_time(p->x, p->y, endpt.x, endpt.y);
				Vertexdata t;
				t.f = f;
				t.mdata = c;
				open.insert(t);
				come_from[c] = p;
			}
			else if (newcost < cost_so_far[c])
			{
				double f = cost_so_far[c] + h_time(c->x, c->y, endpt.x, endpt.y);
				Vertexdata tofind;
				tofind.f = f;
				tofind.mdata = c;
				auto it = open.find(tofind);
				if(it != open.end())
					open.erase(it);

				tofind.f = newcost + h_time(c->x, c->y, endpt.x, endpt.y);
				open.insert(tofind);
				cost_so_far[c] = newcost;
				come_from[c] = p;
			}
		}
		if (cnt % 1000 == 0)
			cout << cnt << endl;
	}
	cout << "Time:" << clock() - t1 << endl;

	double ans = cost_so_far[p];


	Mappointdata *pr = p;
	while (pr != come_from[pr])
	{
		Point t(pr->x, pr->y);
		path.push_front(t);
		pr = come_from[pr];
	}
	FILE* log;
	log = fopen("path.txt", "w");
	for (auto i = path.begin(); i != path.end(); ++i)
		fprintf(log, "%lf,%lf\n", i->x, i->y);
	fclose(log);

	return ans;
}

/*double Predict_Base::predict_delay(Point a,double time)
{
vector<double> loss;
vector<Featurepointdata*> knn;
KNNpointspt(a.x, a.y, time, 1, knn,loss);
double coeffsum = 0;
double predicted_delay = 0;
for (int i = 0; i < 1; ++i)
{
double coeffa = Nfunc(parameter, loss[i]);
coeffsum += coeffa;
predicted_delay += coeffa * knn[i]->delay;
}
predicted_delay /= coeffsum;
return predicted_delay / 3600;
}*/