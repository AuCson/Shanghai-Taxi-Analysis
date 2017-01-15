#define _CRT_SECURE_NO_WARNINGS

#include "Taxidatareader.h"
#include "commonfunc.h"
#include "RTree.h"
#include "RTree_Addon.h"
#include <queue>
#include <random>

using namespace std;


typedef RTREE_WITH_ROAD::Node RNODE;
typedef RTREE_WITH_ROAD::Branch RBRANCH;
typedef RTREE_WITH_ROAD::Rect RRECT;

#define ISNODE 0
#define ISRECT 1
#define ISSEG 2

void TaxireaderHandler::read(int profitcalc)
{
	for (int i = 0; i < filenames.size(); ++i)
	{
		char buf[1000];
		strcpy(buf, filenames[i].c_str());
		FILE* fp = fopen(buf, "r");
		if (fp == NULL) cerr << "无法打开文件:" << filenames[i] << endl;

		//生成 Taxi 对象，并加载单个 Taxi 对应信息并存储

		Taxi *p = readTaxi(fp,1);
		_ASSERT(p);
		Taxis.push_back(p);
		cout << counter++ << endl;
		if (counter % 500 == 0)
			cout << counter << endl;
		fclose(fp);
	}
}

void TaxireaderHandler::write_profit()
{
	profitlog = fopen("Profit_log.csv", "w");
	for (int i = 0; i < Taxis.size(); ++i)
		Taxis[i]->write_profit_log(profitlog);
	fclose(profitlog);
}

void TaxireaderHandler::write_take_log()
{
	take_offlog = fopen("take_log.csv","w");
	for (int i = 0; i < Taxis.size(); ++i)
		Taxis[i]->write_takeoff_log(take_offlog);
	fclose(take_offlog);
}

Taxi* TaxireaderHandler::readTaxi(FILE* fp,int profitcalc)
{
	Taxi* p = new Taxi();
	int id, hour, min, sec, isempty;
	double LONGTITUDE, LATITUDE, prevLO = -1, prevLA = -1, distsum = 0, dis;
	int stat = 0; // stat = 1 为载客开始
	char s[1000];
	char tag[200];
	fgets(s, 1000, fp);
	while (fgets(s, 1000, fp))
	{
		sscanf(s, "%d,%*d-%*d-%*d %d:%d:%d,,%lf,%lf,%*d,%d,%*d,%*d,%*d,%*lf,%*lf,%*d,%s",
			&id, &hour, &min, &sec, &LONGTITUDE, &LATITUDE, &isempty,&tag);
		if (prevLO == -1)
		{
			p->id = id;
			prevLO = LONGTITUDE;
			prevLA = LATITUDE;
			if (!isempty){
				distsum = 0;
				TimePoint point(x, y, hour, min, sec);
				p->startpoint.push_back(point);
				stat = 1;
			}

		}
		dis = distcalc(prevLA, LATITUDE, prevLO, LONGTITUDE);
		p->totaldist += dis;
		if (stat == 0 && tag[10] == 'I') // take in
		{
			stat = 1;
			distsum = 0;
			TimePoint point(x, y, hour, min, sec);
			p->startpoint.push_back(point);
		}
		else if (stat == 1 && tag[10]!='O') // keep going
		{
			prevLO = LONGTITUDE;
			prevLA = LATITUDE;
			distsum += dis;
		}
		else if (stat == 1) //take off
		{
			TimePoint point(x, y, hour, min, sec);
			p->endpoint.push_back(point);
			distsum += dis;
			p->dist.push_back(distsum);
			stat = 0;
		}
		prevLO = LONGTITUDE;
		prevLA = LATITUDE;
	}
	if (stat == 1)
	{
		TimePoint point(x, y, hour, min, sec);
		p->endpoint.push_back(point);
		distsum += dis;
		p->dist.push_back(distsum);
	}
	if(profitcalc)
		p->profitcalc();

	return p;
}

void Taxi::profitcalc()
{
	_ASSERT(startpoint.size() == endpoint.size());
	_ASSERT(endpoint.size() == dist.size());
	for (size_t i = 0; i<dist.size(); ++i)
	{
		Time begin = startpoint[i].time;
		Time end = endpoint[i].time;
		end -= begin;
		if (end.hour == 0 && end.min == 0 && end.sec < 30) //怀疑司机在拨弄计价器
			continue;
		if (dist[i] < 3)
			profit += 14;
		else
		{
			profit += 14 + 2.4*(dist[i] - 3);
		}
	}
	profit -= totaldist * 0.5;
}

void Taxi::write_profit_log(FILE *wp)
{
	fprintf(wp, "%d,%lf\n", id, profit);
}

void Taxi::write_takeoff_log(FILE *wp)
{
	for (size_t i = 0; i < startpoint.size(); ++i)
	{
		double sx, sy, ex, ey;
		co_convert(startpoint[i].LATITUDE, startpoint[i].LONGTITUDE, sx, sy);
		co_convert(endpoint[i].LATITUDE, endpoint[i].LONGTITUDE, ex, ey);
		Time begin = startpoint[i].time;
		Time end = endpoint[i].time;
		end -= begin;
		if (end.hour == 0 && end.min == 0 && end.sec < 30) //怀疑司机在拨弄计价器
			continue;
		fprintf(wp, "%d,%lf,%lf,%d:%d:%d:%d,IN\n", id, sx, sy, 
			startpoint[i].time.day, startpoint[i].time.hour, startpoint[i].time.min, startpoint[i].time.sec);
		
		fprintf(wp, "%d,%lf,%lf,%d:%d:%d:%d,OFF\n", id, ex, ey,
			endpoint[i].time.day, endpoint[i].time.hour, endpoint[i].time.min, endpoint[i].time.sec);
	}
}

void TaxireaderHandler::get_nearest_roadpts(RTREE_WITH_ROAD &tree)
{
	vector<Taxiinoff> info;
	Taxiinoff t;
	
	FILE* fp;
	FILE* fpw;
	int cnt = 0;
	char path[1000];
	char buf[100];
	Point neara,nearb;

	printf("Adjusted take-in/off point path...\n");
	scanf("%s", path);

	fp = fopen(path, "r");
	fpw = fopen("new_takeoff_log.txt", "w");
	
	while (fscanf(fp, "%s", buf) != EOF)
	{
	
		int flg = sscanf(buf, "%d,%lf,%lf%s", &t.id, &t.pt.x, &t.pt.y, t.tag);
		if (flg != 4) break;
		Point newpt = get_nearest_roadpt(t.pt.x, t.pt.y,tree,neara,nearb);
		fprintf(fpw, "%d,%lf,%lf,%lf,%lf,%lf,%lf,%s\n", t.id, newpt.x, newpt.y, neara.x,neara.y,nearb.x,nearb.y,t.tag);
		if (cnt % 1000 == 0) cout << cnt << endl;
		cnt++;
	}
	
	fclose(fpw);
}

struct Quelem
{
	Quelem(){ tag = 0; }
	union
	{
		RNODE* node;
		Maproaddata* data;
	};

	int tag;
	double dist;
};


class Quelem_comp
{
public:
	bool operator()(const Quelem &a, const Quelem &b){ return a.dist > b.dist; }
};

Point TaxireaderHandler::get_nearest_roadpt(double x, double y, RTREE_WITH_ROAD &tree,Point &neara,Point &nearb)
{
	
	return NNpoint(x, y,tree,neara,nearb);
}


Point NNpoint(double x, double y, RTREE_WITH_ROAD &tree, Point &neara,Point &nearb)
{
	RNODE* r = tree.m_root;
	priority_queue<Quelem, vector<Quelem>, Quelem_comp> prque;
	Quelem q;
	q.tag = ISNODE; q.dist = 0; q.node = r;
	prque.push(q);
	Point nearest;

	while (!prque.empty())
	{
		q = prque.top();
		prque.pop();
		if (q.tag == ISSEG || q.tag == ISRECT)
		{
			int flg = 0;
			double dist = q.dist; //when seg
			if (q.tag == ISRECT && !prque.empty())
			{
				double segstartx = q.data->start.x;
				double segendx = q.data->end.x;
				double segstarty = q.data->start.y;
				double segendy = q.data->end.y;
				dist = p2segdist2(x, y, segstartx, segstarty, segendx, segendy);
				if (dist > prque.top().dist)
					flg = 1;
			}
			if (flg)
			{
				Quelem t;
				t.tag = ISSEG;
				t.data = q.data;
				t.dist = dist;
				prque.push(t);
			}
			else
			{
				//找到最近邻点
				nearest = p2segshadow(x, y, q.data->start.x, q.data->start.y, q.data->end.x, q.data->end.y);


				neara.x = q.data->start.x;
				neara.y = q.data->start.y;

				nearb.x = q.data->end.x;
				nearb.y = q.data->end.y;

				return nearest;
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

				//计算到Rect的mindist

				double segstartx = q.node->m_branch[i].m_rect.m_min[0];
				double segendx = q.node->m_branch[i].m_rect.m_max[0];
				double segstarty = q.node->m_branch[i].m_rect.m_min[1];
				double segendy = q.node->m_branch[i].m_rect.m_max[1];

				double mindist;
				if ((x <= segstartx&&x >= segendx || x <= segendx&&x >= segstartx) &&
					(y <= segendy&&y >= segstarty || y <= segstarty && y >= segendy))
					t.dist = 0;
				else{
					double tmin[4];
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

void TaxireaderHandler::statistic(RTREE_WITH_ROAD &treerd)
{
	cout << "Begin of statistic" << endl;
	for (size_t i = 0; i < filenames.size(); ++i)
	{
		char buf[1000];
		FILE* wfp = fopen("Statistic_Velocity.csv", "a");

		strcpy(buf, filenames[i].c_str());
		FILE* fp = fopen(buf, "r");
		statistic_single(fp, treerd, wfp);
		fclose(fp);
		fclose(wfp);
		cout << i << endl;
	}

}

void TaxireaderHandler::statistic_single(FILE *fp, RTREE_WITH_ROAD & treerd, FILE* log)
{
	double lo, la, speed;
	int  hour, min, sec, prevh, prevm, prevs;
	double prevLO = -1, prevLA = -1;
	int isempty;
	int stat = 0;
	Point va, vb, lva1, lva2, lvb1, lvb2;
	double x, y;
	char tag[200];
	char buf[1000];

	fgets(buf, 1000, fp);
	while (fgets(buf, 1000, fp))
	{
		sscanf(buf, "%*d,%*d-%*d-%*d %d:%d:%d,,%lf,%lf,%*d,%d,%*d,%*d,%*d,%lf,%*lf,%*d,%s",
			&hour, &min, &sec, &lo, &la, &isempty, &speed, tag);
		if (stat == 1)
		{
			co_convert(la, lo, x, y);
			NNpoint(x, y, treerd, va, vb);
			fprintf(log, "%lf,%lf,%lf,%lf,%lf,%d:%d:%d\n", va.x, va.y, vb.x, vb.y, speed, hour, min, sec);
		}
		if (prevLO == -1)
		{
			prevLO = lo;
			prevLA = la;
			if (!isempty)
				stat = 1;
		}
		if (stat == 0 && tag[10] == 'I') // take in
		{
			stat = 1;
		}
		else if (stat == 1 && tag[10] == 'O') //take off
		{
			stat = 0;
		}
		prevLO = lo;
		prevLA = la;
	}

}