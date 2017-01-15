#include "RTree_Addon.h"
using namespace std;

static int roadidcnt = 0;

bool _cdecl data_store_callback(MAPDATA& data, void *arg)
{
	vector<MAPDATA*> *vec = (vector<MAPDATA*> *) arg;
	vec->push_back(&data);
	return true;
}

void Insert_adj(RTree<Mappointdata*, double, 2> &tree, const double a_min[2], const double a_max[2], Mappointdata* a_dataId, double eps)
{
	int has_same_point;
	double t_min[2];
	double t_max[2];
	vector<Mappointdata**> container;
	for (int i = 0; i < 2; ++i)
	{
		t_min[i] = a_min[i];
		t_max[i] = a_max[i];
	}

	has_same_point = tree.Search(t_min, t_max, data_store_callback, &container);
	if (has_same_point)
	{
		for (int t = 0; t < has_same_point; ++t)
		{
			MAPDATA* p = container[t];
			for (size_t i = 0; i < a_dataId->edge.size(); ++i)
			{
				int flg = 0;
				if (p == NULL) break;
				for (size_t j = 0; j < (*p)->edge.size(); ++j)
				{
					if (a_dataId->edge[i].x == (*p)->edge[j].x && a_dataId->edge[i].y == (*p)->edge[j].y)
					{
						flg = 1;
						break;
					}
				}
				if (!flg)
					(*p)->edge.push_back(a_dataId->edge[i]);
				_ASSERT((*p)->edge.size() < 20);
			}
		}
	}
	else
	{
		a_dataId->id = roadidcnt++;
		tree.Insert(t_min, t_max, a_dataId);
	}
}


