#ifndef TRAJECTORY_H 
#define TRAJECTORY_H 

#include "dStarLite.hpp"
#include <vector>
#include <map>
#include <vector>
#include <cstdlib>
#include <utility>
#include <algorithm>
#include <cmath>
#include <bits/stdc++.h>
#include <limits>
#include "mapGeneration.hpp"


std::vector<Node> pathTreatment(std::vector<Node> path); 
bool sensorTreatment(int enemyX, int enemyY, int enemyWidth, 
	std::vector<std::vector<int>> mapVector, std::vector<Node> path); 

#endif // DSTARTLITE_H_INCLUDED