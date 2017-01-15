#include "AnalysisBase.h"
#include <algorithm>
#include <queue>
#include "Taxidatareader.h"
#include "RTree_Addon.h"
using namespace std;

#define EPS 0.00001
#define SPEED 40.00

bool querysort(Taxiquery *a,Taxiquery *b)
{
	return a->time < b->time;
}


void Analysis_Base::analysis()
{
	string querypath;
	cout << "Path of queries..." << endl;
	cin >> querypath;
	
	read_query(querypath);
	FILE* log;
	log = fopen("Analysis_when_parking.csv", "a");
	fprintf(log, "ID,ProfitA,ProfitB,ProfitC,ProfitD\n");
	fclose(log);
	
	for (size_t i = 0; i < files.size(); ++i)
	{
		log = fopen("Analysis_when_parking.csv", "a");
		FILE* fpt;
		char filename[1000];
		int id;
		double firstlo, firstla, firstx, firsty;
		Point startroadpta, startroadptb;
		Point startpt;
		Anainfo info;

		strcpy(filename, files[i].c_str());
		fpt = fopen(filename, "r");
		fscanf(fpt, "%*s");
		fscanf(fpt, "%d,%*d-%*d-%*d %*d:%*d:%*d,,%lf,%lf,%*s",
			&id,&firstlo, &firstla);
		fclose(fpt);
		co_convert(firstla, firstlo, firstx, firsty);
		NNpoint(firstx, firsty, roadtreer,startroadpta,startroadptb);
		startpt.x = firstx;
		startpt.y = firsty;
		
		info = ana_single(startpt, startroadpta,startroadptb);
		info.id = id;
		fprintf(log, "%d,%lf,%lf,%lf,%lf\n", info.id, info.profit[0], info.profit[1], info.profit[2], info.profit[3]);
		//infos.push_back(info);

		cout << i << endl << endl << endl << endl << endl;
		fclose(log);
	}
	
}

void Analysis_Base::read_query(string path)
{
	char buf[1000];
	strcpy(buf, path.c_str());
	FILE* fp = fopen(buf, "r");
	Time time;
	int cnt = 0;
	while (fscanf(fp,"%s", buf) != EOF)
	{
		if (cnt % 10000 == 0) cout << cnt << endl;
		cnt++;
		Taxiquery* q = new Taxiquery(); // id => q->id
		int t = sscanf(buf, "%d,%lf,%lf,%lf,%lf,%lf,%lf,,%d:%d:%d:%d,%*s", &q->taxiid, &q->onroadsrc.x, &q->onroadsrc.y, 
			&q->vsrca.x, &q->vsrca.y, &q->vsrcb.x,&q->vsrcb.y,&q->time.day,&q->time.hour, &q->time.min, &q->time.sec);
		if (t != 11)
			break;
		fscanf(fp, "%s", buf);
		sscanf(buf, "%d,%lf,%lf,%lf,%lf,%lf,%lf,,%d:%d:%d:%d,%*s", &q->taxiid, &q->onroaddst.x, &q->onroaddst.y,
			&q->vdsta.x, &q->vdsta.y, &q->vdstb.x, &q->vdstb.y, &q->time.day, &q->time.hour, &q->time.min, &q->time.sec);
		query.push_back(q);	

		//insertedge(q->onroadsrc, q->vsrca, q->vsrcb);
		//insertedge(q->onroaddst, q->vdsta, q->vdstb);
	}
	sort(query.begin(), query.end(), querysort);
	Time bias(0, 5, 0);
	for (size_t i = 0; i < query.size(); ++i)
	{
		query[i]->time -= bias;
	}
}


Mappointdata* Analysis_Base::insertedge(Point onroad, Point va, Point vb)
{
	Mappointdata* p = new Mappointdata;
	double amin[2], amax[2];
	p->x = onroad.x; p->y = onroad.y;

	p->edge.push_back(va);
	p->edge.push_back(vb);
	amin[0] = p->x; amax[0] = p->x;
	amin[1] = p->y; amax[1] = p->y;
	Insert_adj(roadtree, amin, amax, p, 0);

	vector<Mappointdata**> container;
	amin[0] = va.x - EPS; amin[1] = va.y - EPS;
	amax[0] = va.x - EPS; amax[1] = va.y + EPS;
	roadtree.Search(amin, amax, data_store_callback, &container);
	for (int i = 0; i < container.size(); ++i)
	{
		if (find((*container[i])->edge.begin(), (*container[i])->edge.end(), onroad) == ((*container[i])->edge).end())
			(*container[i])->edge.push_back(onroad);
	}

	container.clear();
	amin[0] = vb.x - EPS; amin[1] = vb.y - EPS;
	amax[0] = vb.x - EPS; amax[1] = vb.y + EPS;
	roadtree.Search(amin, amax, data_store_callback, &container);
	for (int i = 0; i < container.size(); ++i)
	{
		if (find((*container[i])->edge.begin(), (*container[i])->edge.end(), onroad) == ((*container[i])->edge).end())
			(*container[i])->edge.push_back(onroad);
	}
	return p;
}

void Analysis_Base::addqtotree(Taxiquery* q)
{
	double tmin[2];
	double tmax[2];
	tmin[0] = tmax[0] = q->onroadsrc.x;
	tmin[1] = tmax[1] = q->onroadsrc.y;

	querytree.Insert(tmin, tmax, q);
}

void Analysis_Base::removequery(Taxiquery* q)
{
	double tmin[2];
	double tmax[2];
	tmin[0] = tmax[0] = q->onroadsrc.x;
	tmin[1] = tmax[1] = q->onroadsrc.y;
	querytree.Remove(tmin, tmax, q);
}

Analysis_Base::Anainfo Analysis_Base::ana_single(Point startpt,Point startva,Point startvb)
{
	const Time lim[4] = { Time(6, 0, 0), Time(12, 0, 0), Time(18, 0, 0), Time(1, 0, 0, 0) };
	int lasti = 0;
	double totaldist[4] = {0};
	double profit[4] = {0};
	Time currenttime;
	vector<list<Point> > paths;
	querytree.RemoveAll();

	for (lasti = 0;; ++lasti)
	{
		Time flg(0, 0, 0);
		if (query[lasti]->time < flg)
			addqtotree(query[lasti]);
		else
			break;
	}
	currenttime = Time(0, 0, 0);

	for (int tp = 0; tp < 4; )
	{
		int timeoverflg = 0;
		Taxiquery* nearest = getnearest(startpt);
		double dist2 = p2pdist2(startpt.x, startpt.y, nearest->onroadsrc.x, nearest->onroadsrc.y);
		if (dist2>3.00*3.00)
		{
			while (1)
			{
				Taxiquery* t = query[lasti++];
				if (lim[tp] < t->time || lasti >= query.size())
				{
					timeoverflg = 1;
					break;
				}
				dist2 = p2pdist2(startpt.x, startpt.y, t->onroadsrc.x, t->onroadsrc.y);
				if (dist2 < 3.00 * 3.00)
				{
					nearest = t;
					break;
				}
				addqtotree(t);
			}
		}
		if (timeoverflg != 1)
		{
			list<Point> path;
			Point bests, beste;
			removequery(nearest);
			getbestpt(startva, startvb, nearest->vsrca, nearest->vsrcb, bests, beste);
			double mindist = Astar(bests.x,bests.y,beste.x,beste.y, path,0);
			cout << "dist1  " << mindist << endl;
			mindist += p2pdist(bests, startpt);
			mindist += p2pdist(beste, nearest->onroadsrc);
			paths.push_back(path);

			getbestpt(nearest->vsrca, nearest->vsrcb, nearest->vdsta,nearest->vdstb,bests, beste);
			double profitdist = Astar(bests.x, bests.y, beste.x, beste.y, path, 0);
			cout << "dist2  " << profitdist << endl;
			profitdist += p2pdist(bests, nearest->onroadsrc);
			profitdist += p2pdist(beste, nearest->onroaddst);

			paths.push_back(path);
		
			profit[tp] += earncalc(profitdist);
			totaldist[tp] += mindist + profitdist;
			
			double hourcost = (mindist + profitdist) / SPEED;
			int sec = (int)(3600 * hourcost);
			Time timecost;
			timecost.hour = sec / 3600;
			timecost.min = sec % 3600 / 60;
			timecost.sec = sec % 3600 % 60;
			currenttime += timecost;

			startpt = nearest->onroaddst;
			startva = nearest->vdsta;
			startvb = nearest->vdstb;

			if (lim[tp] < currenttime)
			{
				timeoverflg = 1;
			}
		
			while (lasti < query.size() && query[lasti]->time<currenttime )
			{
				addqtotree(query[lasti]);
				++lasti;
				if (lasti >= query.size())
					timeoverflg = 1;
			}
		}
		if (timeoverflg == 1)
			tp++;
	}

	Anainfo info;
	for (int i = 0; i < 4; ++i)
	{
		info.profit[i] = profit[i] - 0.5*totaldist[i];
	}
	return info;
}


typedef RTree<Taxiquery*, double, 2> RTREE_WITH_QUERY;
typedef RTREE_WITH_QUERY::Node RNODE;
typedef RTREE_WITH_QUERY::Branch RBRANCH;
typedef RTREE_WITH_QUERY::Rect RRECT;

#define ISNODE 0
#define ISRECT 1
#define ISPOINT 2

struct Quelem
{
	Quelem(){ tag = 0; }
	union
	{
		RNODE* node;
		Taxiquery* data;
	};

	int tag;
	double dist;
};

class Quelem_comp
{
public:
	bool operator ()(const Quelem &a, const Quelem &b)
	{
		return a.dist > b.dist;
	}
};

Taxiquery* Analysis_Base::getnearest(Point startp)
{
	RNODE* r = querytree.m_root;
	priority_queue<Quelem, vector<Quelem>, Quelem_comp> prque;
	Quelem q;
	q.tag = ISNODE; q.dist = 0; q.node = r;
	prque.push(q);
	Point nearest;

	while (!prque.empty())
	{
		q = prque.top();
		prque.pop();
		if (q.tag == ISPOINT || q.tag == ISRECT)
		{
			int flg = 0;
			double dist = q.dist; //when seg
			if (q.tag == ISRECT && !prque.empty())
			{
				double ptx = q.data->onroadsrc.x;
				double pty = q.data->onroadsrc.y;
				dist = p2pdist2(startp.x, startp.y, ptx,pty);
				if (dist > prque.top().dist)
					flg = 1;
			}
			if (flg)
			{
				Quelem t;
				t.tag = ISPOINT;
				t.data = q.data;
				t.dist = dist;
				prque.push(t);
			}
			else
			{
				return q.data;
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

				//¼ÆËãµ½RectµÄmindist

				double segstartx = q.node->m_branch[i].m_rect.m_min[0];
				double segendx = q.node->m_branch[i].m_rect.m_max[0];
				double segstarty = q.node->m_branch[i].m_rect.m_min[1];
				double segendy = q.node->m_branch[i].m_rect.m_max[1];

				double mindist;
				if ((startp.x <= segstartx&&startp.x >= segendx || startp.x <= segendx&&startp.x >= segstartx) &&
					(startp.y <= segendy&&startp.y >= segstarty || startp.y <= segstarty && startp.y >= segendy))
					t.dist = 0;
				else{
					double tmin[4];
					double x = startp.x;
					double y = startp.y;
					mindist = tmin[0] = p2segdist2(x, y, segstartx, segstarty, segendx, segstarty);
					tmin[1] = p2segdist2(x, y, segstartx, segendy, segendx, segendy);
					tmin[2] = p2segdist2(x, y, segstartx, segstarty, segstartx, segendy);
					tmin[3] = p2segdist2(x, y, segendx, segstarty, segendx, segendy);
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