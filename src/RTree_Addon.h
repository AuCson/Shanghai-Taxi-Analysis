#pragma once
#ifndef _GUARD_RTREEADDON_H
#define _GUARD_RTREEADDON_H

#include "RTree.h"
#include "typedefine.h"


bool _cdecl data_store_callback(MAPDATA& data, void *arg);
void Insert_adj(RTree<Mappointdata*, double, 2> &tree, const double a_min[2], const double a_max[2], Mappointdata* a_dataId, double eps);

#endif
#pragma once