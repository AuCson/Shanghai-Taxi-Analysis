#include "AnalysisBase.h"
#include <algorithm>
#include <queue>
#include "Taxidatareader.h"
#include "RTree_Addon.h"
using namespace std;

#define NSPEED 10.0
#define FSPEED 40.0
#define EPS 0.00001
#define UPDATESEC updatesec

void timeupdate(Time &currenttime,int addsec)
{
	Time timecost;
	timecost.hour = addsec / 3600;
	timecost.min = addsec % 3600 / 60;
	timecost.sec = addsec % 3600 % 60;
	currenttime += timecost;
}

void Analysis_Base::analysis_mov()
{
	string querypath;
	cout << "Path of queries..." << endl;
	cin >> querypath;

	read_query(querypath);
	FILE* log;
	log = fopen("Analysis_when_moving.csv", "a");
	fprintf(log, "ID,ProfitA,ProfitB,ProfitC,ProfitD\n");
	fclose(log);
	for (size_t i = 0; i < files.size(); ++i)
	{
		log = fopen("Analysis_when_moving.csv", "a");

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
			&id, &firstlo, &firstla);

		co_convert(firstla, firstlo, firstx, firsty);
		NNpoint(firstx, firsty, roadtreer, startroadpta, startroadptb);
		startpt.x = firstx;
		startpt.y = firsty;

		info = ana_single_m(startpt, startroadpta, startroadptb);
		info.id = id;
		fprintf(log, "%d,%lf,%lf,%lf,%lf\n", info.id, info.profit[0], info.profit[1], info.profit[2], info.profit[3]);
		//infos.push_back(info);
		fclose(log);
		fclose(fpt);
		cout << i<<"========================" << endl << endl << endl << endl << endl;
	}

}

void Analysis_Base::getbestpt(Point sa, Point sb, Point ea, Point eb, Point &bests, Point &beste)
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

Analysis_Base::Anainfo Analysis_Base::ana_single_m(Point startpt, Point startva, Point startvb)
{
	const Time lim[4] = { Time(6, 0, 0), Time(12, 0, 0), Time(18, 0, 0), Time(1, 0, 0, 0) };
	int lasti = 0;
	int hasbetterflg = 0;
	double totaldist[4] = { 0 };
	double profit[4] = { 0 };
	Time currenttime;
	Point now;
	list<Point> path;

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

	for (int tp = 0; tp < 4;)
	{
		int timeoverflg = 0;
		Taxiquery* nearest = getnearest(startpt);
		double dist = p2pdist(startpt.x, startpt.y, nearest->onroadsrc.x, nearest->onroadsrc.y);
		int tryt = 0;
		int foundflg = 0;
		while (dist >3.00)
		{
			//开始巡游，目标点为Nearest
			
			tryt++;
			Point bests, beste;
			cout << "Try" << endl;
			if (lim[tp] < currenttime)
			{
				timeoverflg = 1;
				break;
			}
			cout << currenttime.hour <<'\t' <<currenttime.min <<'\t'<< currenttime.day << endl;
			getbestpt(startva, startvb, nearest->vsrca, nearest->vsrcb, bests, beste);
			double mindist = Astar(bests.x, bests.y, beste.x, beste.y, path,1);
			if (mindist == -1)
			{
				cerr << "搜索失败，可能有非连通区域" << endl;
				timeoverflg = 1;
				break;
			}
			path.pop_front();

			now = startpt;
			while (!path.empty())
			{
				Point nxt = path.front();
				path.pop_front();
				double needsec = (int)(3600 * p2pdist(nxt, startpt) / NSPEED);
				double addsec = 0;
				double updatesec = needsec > 300 ? needsec / 20 : 15;
				Time ttime;
				while (addsec < needsec)
				{
					ttime = currenttime;
					addsec += ((addsec + UPDATESEC) > needsec ? needsec - addsec : UPDATESEC);
					now.x = startpt.x + (nxt.x - startpt.x) * (addsec / needsec);
					now.y = startpt.y + (nxt.y - startpt.y) * (addsec / needsec);
					totaldist[tp] += p2pdist(startpt, now);
					Taxiquery* t = getnearest(now);
					timeupdate(ttime, (addsec + UPDATESEC) > needsec ? needsec : addsec + UPDATESEC);
					while (lasti != query.size() && query[lasti]->time < ttime)
						addqtotree(query[lasti++]);
					if (lim[tp] < ttime)
					{
						timeoverflg = 1; 
						currenttime = ttime;
						break;
					}
					dist = p2pdist(now, nearest->onroadsrc);
					if (dist < 3.00)
					{
						startpt = now;
						foundflg = 1;
						break;
					}
					if (t != nearest)
					{
						//update
						nearest = t;
						startpt = now;
						dist = p2pdist(t->onroadsrc, now);
						break;
					}

				}
				currenttime = ttime;
				if (lim[tp] < currenttime)
				{
					timeoverflg = 1;
					break;
				}
				if (foundflg)
					break;
				startpt = nxt;
			}
		}
		if (timeoverflg != 1)
		{
			list<Point> path;
			Point bests, beste;
			removequery(nearest);
			cout << "Begin" << endl;
			getbestpt(startva, startvb, nearest->vsrca, nearest->vsrcb, bests, beste);
			double mindist = Astar(bests.x, bests.y, beste.x, beste.y, path,1);
			mindist += p2pdist(bests, startpt);
			mindist += p2pdist(beste, nearest->onroadsrc);
			cout << mindist << endl;

			getbestpt(nearest->vsrca, nearest->vsrcb, nearest->vdsta, nearest->vdstb, bests, beste);
			double profitdist = Astar(bests.x, bests.y, beste.x, beste.y, path,1);
			profitdist += p2pdist(bests, nearest->onroadsrc);
			profitdist += p2pdist(beste, nearest->onroaddst);

			cout << profitdist << endl;

			profit[tp] += earncalc(profitdist);
			totaldist[tp] += mindist + profitdist;

			double hourcost = (mindist + profitdist) / FSPEED;
			int sec = (int)(3600 * hourcost);
			timeupdate(currenttime, sec);
			startpt = nearest->onroaddst;
			startva = nearest->vdsta;
			startvb = nearest->vdstb;

			if (lim[tp] < currenttime)
			{
				timeoverflg = 1;
			}

			while (lasti < query.size() && query[lasti]->time<currenttime)
			{
				addqtotree(query[lasti]);
				++lasti;
				
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
