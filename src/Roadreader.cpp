#include "Roadreader.h"
using namespace std;

RoadreaderHandler::RoadreaderHandler(string path_)
{
	strcpy(path,path_.c_str());
	fp = fopen(path, "r");
	if (!fp)
		cerr << "无法打开文件" << endl;
	counter = 0;
}

void RoadreaderHandler::read_point_toTree()
{
	char s[1000];
	int previsempty = 1, suc;
	double prevx, prevy;
	double x, y;
	double la, lo;
	Mappointdata* prevp;
	fscanf(fp, "%s", s);
	int cnt = 0, sum = 0, t = 0;
	while (fscanf(fp, "%s", s) != EOF)
	{
		if (cnt++ % 10000 == 0) { cout << cnt - 1 << '\t' << clock() - t << endl; sum += clock() - t; t = clock(); }
		if (s[0] != '_'&&previsempty) // start reading data of a single road
		{
			sscanf(s, "%lf,%lf", &la, &lo);
			co_convert(la, lo, x, y);
			prevp = new Mappointdata();
			prevx = prevp->x = x;	prevy = prevp->y = y;
			previsempty = 0;
		}
		else if (s[0] == '_')
		{
			double bound_min[2] = { x, y };
			double bound_max[2] = { x, y };
			Insert_adj(tree, bound_min, bound_max, prevp, 0.00001);
			previsempty = 1;
		}
		else
		{
			Mappointdata* p = new Mappointdata();
			sscanf(s, "%lf,%lf", &la, &lo);
			co_convert(la, lo, x, y);
			p->x = x;	p->y = y;
			Point t(x, y);
			Point prevpt(prevx, prevy);
			prevp->edge.push_back(t);
			p->edge.push_back(prevpt);
			double bound_min[2] = { prevx, prevy };
			double bound_max[2] = { prevx, prevy };
			Insert_adj(tree, bound_min, bound_max, prevp, 0.00001);
			prevx = x;	prevy = y;	prevp = p;
			previsempty = 0;
		}
	}
	cout << sum << endl;
}

void RoadreaderHandler::read_road_toTree()
{
	char s[1000];
	int previsempty = 1;
	double prevx, prevy;
	double la, lo;
	double x, y;
	fscanf(fp, "%s", s);
	int cnt = 0, sum = 0, t = 0;
	while (fscanf(fp, "%s", s) != EOF)
	{
		if (cnt++ % 10000 == 0) { cout << cnt - 1 << '\t' << clock() - t << endl;  t = clock(); }
		if (s[0] != '_'&&previsempty) // start reading data of a single road
		{
			sscanf(s, "%lf,%lf", &la, &lo);
			co_convert(la, lo, x, y);
			prevx = x;	prevy = y;
			previsempty = 0;
		}
		else if (s[0] == '_')
		{
			previsempty = 1;
		}
		else
		{
			Maproaddata* p = new Maproaddata();
			sscanf(s, "%lf,%lf", &la, &lo);
			co_convert(la, lo, x, y);
			p->start = Point(x, y);
			p->end = Point(prevx, prevy);

			double bound_min[2] = { x<prevx?x:prevx, y<prevy?y:prevy };
			double bound_max[2] = { x>prevx?x:prevx, y>prevy?y:prevy };
			rdtree.Insert(bound_min, bound_max, p);
			prevx = x;	prevy = y;
			previsempty = 0;
			sum++;
		}
	}
	cout << sum << endl;
}