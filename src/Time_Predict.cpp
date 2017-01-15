#include "Travel_time_predict.h"
#include <string>
#include <vector>
#include <queue>
#include <cmath>
using namespace std;
#define ISNODE 0
#define ISRECT 1
#define ISDATA 2
#define PARA (1.0/3600)

typedef RTREE_WITH_FEA::Node RNODE;
void getbestpt(Point sa, Point sb, Point ea, Point eb, Point &bests, Point &beste);
double Predict_Base::predict(Point startpt, Point endpt,int hour,int min,int sec)
{
	double secs = 3600 * hour + 60 * min + sec;
	Point neara, nearb,vsrca,vsrcb,vdsta,vdstb,rds,rde;
	list<Point> path;
	NNpoint(startpt.x, startpt.y, rdtree, vsrca, vsrcb);
	NNpoint(endpt.x, endpt.y, rdtree, vdsta, vdstb);
	getbestpt(vsrca, vsrcb, vdsta, vdstb, rds, rde);

	double time = Predict_road_time(startpt, rds, secs/3600);
	time += Astar_time(rds, rde, secs/3600 + time,path);
	time += Predict_road_time(endpt, rde, secs / 3600 + time);

	time *= 3600;
	int h = (int)time / 3600;
	int m = (int)time % 3600 / 60;
	int s = (int)time % 3600 % 60;
	cout << "Complete!----------" << endl;
	cout << endl;
	cout << h <<" hour "<<m<<" min "<< s << " sec"<<endl;
	return time;
}

double Predict_Base::Nfunc(double parameter, double loss)
{
	return (1.0 / (parameter * sqrt(2 * 3.1415926535))) * pow(2.718281828459, (-(loss) * (loss)) / (2 * parameter *parameter));
}

void Predict_Base::loaddataset()
{
	FILE *fp = fopen(datasetdir, "r");
	char buf[1000];
	int hour, min, sec;
	int cnt =0;
	cout << "loading dataset 'Velocity'" << endl;
	while (fscanf(fp,"%s", buf) != EOF)
	{
		cnt++;
		if (cnt == 100000) break;
		if (cnt % 10000 == 0) cout << cnt << endl;
		if (buf[0] == '\0') break;
		Featurepointdata* p = new Featurepointdata();
		sscanf(buf, "%lf,%lf,%lf,%lf,%lf,%d:%d:%d", &p->a.x, &p->a.y, &p->b.x, &p->b.y, &p->speed, &hour, &min, &sec);
		double totalsec = 3600 * hour + 60 * min + sec;
		p->sec = (int)totalsec;
		double amin[3] = { p->a.x < p->b.x ? p->a.x : p->b.x, p->a.y < p->b.y ? p->a.y : p->b.y, totalsec };
		double amax[3] = { p->a.x > p->b.x ? p->a.x : p->b.x, p->a.y > p->b.y ? p->a.y : p->b.y, totalsec };

		ftree.Insert(amin, amax, p);
	}
	fclose(fp);
}


struct Quelem
{
	Quelem(){ tag = 0; }
	union
	{
		RNODE* node;
		Featurepointdata* data;
	};
	int tag;
	double dist;
};


class Quelem_comp
{
public:
	bool operator()(const Quelem &a, const Quelem &b){ return a.dist > b.dist; }
};


void Predict_Base::KNNpoints(double x, double y, double time, int k, vector<Featurepointdata*> &rtn,vector<double> &losses,double predictedmax)
{
	RNODE *r = ftree.m_root;
	priority_queue<Quelem, vector<Quelem>, Quelem_comp> prque;
	Quelem q;
	q.tag = ISNODE; q.dist = 0; q.node = r;
	prque.push(q);
	while (!prque.empty())
	{
		q = prque.top();
		prque.pop();
		if (q.tag == ISDATA || q.tag == ISRECT)
		{
			int flg = 0;
			double dist = q.dist; 
			if (q.tag == ISRECT && !prque.empty())
			{
				double segstartx = q.data->a.x;
				double segendx = q.data->b.x;
				double segstarty = q.data->a.y;
				double segendy = q.data->b.y;
				double segdist2 = p2segdist2(x, y, segstartx, segstarty, segendx, segendy); 
				double gap = abs(time - q.data->sec) > 43200 ? 86400 - abs(time - q.data->sec) : abs(time - q.data->sec);
				gap *= PARA;
				double dist = sqrt(segdist2 + gap * gap);
				if (dist > prque.top().dist)
					flg = 1;
			}
			if (flg)
			{
				Quelem t;
				t.tag = ISDATA;
				t.data = q.data;
				t.dist = dist;
				prque.push(t);
			}
			else
			{
				rtn.push_back(q.data);
				losses.push_back(dist);
				if (rtn.size() == k)
					return;
			}
		}
		else if (q.tag == ISNODE)
		{
			for (int i = 0; i < q.node->m_count; ++i)
			{
				Quelem t;
				t.tag = q.node->IsLeaf() ? ISRECT : ISNODE;
				if (t.tag == ISNODE)
					t.node = q.node->m_branch[i].m_child;
				else if (t.tag == ISRECT)
					t.data = q.node->m_branch[i].m_data;

				double segstartx = q.node->m_branch[i].m_rect.m_min[0];
				double segendx = q.node->m_branch[i].m_rect.m_max[0];
				double segstarty = q.node->m_branch[i].m_rect.m_min[1];
				double segendy = q.node->m_branch[i].m_rect.m_max[1];

				double mindist;
				if ((x <= segstartx&&x >= segendx || x <= segendx&&x >= segstartx) &&
					(y <= segendy&&y >= segstarty || y <= segstarty && y >= segendy))
				{
					t.dist = abs(q.node->m_branch[i].m_rect.m_min[2] - time) > 43200 ?
						86400 - abs(q.node->m_branch[i].m_rect.m_min[2] - time) : abs(q.node->m_branch[i].m_rect.m_min[2] - time);
					t.dist *= PARA;
				}
				else{
					double tmin[4];
					double timegap = q.node->m_branch[i].m_rect.m_min[2] - time;
					if (abs(timegap) > 43200)
						timegap = 86400 - abs(timegap);
					timegap *= PARA;
					mindist = tmin[0] = sqrt(p2segdist2(x, y, segstartx, segstarty, segendx, segstarty) + timegap * timegap);
					tmin[1] = sqrt(p2segdist2(x, y, segstartx, segendy, segendx, segendy) + timegap * timegap);
					tmin[2] = sqrt(p2segdist2(x, y, segstartx, segstarty, segstartx, segendy) + timegap * timegap);
					tmin[3] = sqrt(p2segdist2(x, y, segendx, segstarty, segendx, segendy) + timegap * timegap);
					for (int j = 0; j < 4; ++j)
					if (tmin[j] < mindist)
						mindist = tmin[j];
					t.dist = mindist;
				}
				prque.push(t);
			}

		}
	}
	_ASSERT(0);

}

void getbestpt(Point sa, Point sb, Point ea, Point eb, Point &bests, Point &beste)
{
	double dis1 = p2pdist2(sa.x, sa.y, ea.x, ea.y);
	double mint = dis1;
	bests = sa; beste = ea;
	double dis2 = p2pdist2(sa.x, sa.y, eb.x, eb.y);
	if (dis2 < mint) { mint = dis2; bests = sa; beste = eb; }
	double dis3 = p2pdist2(sb.x, sb.y, ea.x, ea.y);
	if (dis3 < mint) { mint = dis3; bests = sb; beste = ea; }
	double dis4 = p2pdist2(sb.x, sb.y, eb.x, eb.y);
	if (dis4 < mint) { mint = dis4; bests = sb; beste = eb; }
}

