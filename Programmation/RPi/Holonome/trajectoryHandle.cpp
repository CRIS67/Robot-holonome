#include "trajectoryHandle.hpp"

/*
Detects where the angles of the trajectory change and thus 
simplifies the trajectory 
*/
std::vector<Node> pathTreatment(std::vector<Node> path)
{	
	std::vector<Node> simplifiedPath;

	for(int i = 0; i<path.size()-2; i++)
	{
		Node tmpNode = path.at(i); 
		Node nextNode = path.at(i+1); 
		Node furtherNode = path.at(i+2); 

		int xVector = nextNode.coord.first - tmpNode.coord.first; 
		int yVector = nextNode.coord.second - tmpNode.coord.second; 

		int xVectorF = tmpNode.coord.first - furtherNode.coord.first; 
		int yVectorF = tmpNode.coord.second - furtherNode.coord.second; 

		double firstAngle = atan2(yVector,xVector); 
		double secondAngle = atan2(yVectorF, xVectorF); 

		if(firstAngle != secondAngle)
		{
			simplifiedPath.push_back(nextNode); 
		}

	}

	return simplifiedPath; 
}

bool sensorTreatment(int enemyX, int enemyY, int enemyWidth, 
	std::vector<std::vector<int>> mapVector, std::vector<Node> path)
{
	clearMap(mapVector); // clears the map 
	generateMap(mapVector, mapVector.size() , mapVector.at(0).size()); // initializes the original map 
	createRectangle( enemyX, enemyY, enemyWidth, enemyWidth,mapVector); // generates the obstacle zone 

	// Check if the path passes through the obsctale 
	for(int i = 0; i< path.size(); i++)
	{
		int x = path.at(i).coord.first; 
		int y = path.at(i).coord.second; 

		if(mapVector[x][y] == 1)
		{
			return true; 
		}
	}

	return false; 
}