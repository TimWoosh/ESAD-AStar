// This version contains a bug and therefore finds wrong paths...
#include <iostream>
#include <set>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <sstream>

/**
 * This template requires an implemented std::ostream& operator<<(std::ostream& os,const T& t)
 */
template<typename T>
inline std::string anyToString(const T& x)
{
	std::ostringstream o;
	if (!(o << x))
	{
		throw std::invalid_argument(std::string("anyToString(") + typeid(x).name() + ")");
	}
	return o.str();
}
/**
 * Vertex is a location on the grid
 */
struct Vertex
{
		//Initiate the Vertex by setting the coördinates. Set the starting values for the Cost
		Vertex(int anX, int anY) :x(anX),y(anY),actualCost(0.0),heuristicCost(0.0){}


		//If the heuristic cost for this Vertex is less than that of aVertex, return true
		bool operator<(const Vertex& aVertex) const
		{
			if(heuristicCost < aVertex.heuristicCost) return true;
			//if(heuristicCost == aVertex.heuristicCost) return actualCost > aVertex.actualCost; // less uncertainty
			return false;
		}

		//Return true if the location of this Vertex is identical to the location of aVertex
		bool operator==(const Vertex& aVertex) const
		{
			return x == aVertex.x && y == aVertex.y;
		}
		//Show the Id (x y coordinates) of this Vertex in a string
		std::string getId() const
		{
			return anyToString(x) + "-" + anyToString(y);
		}

		int x;
		int y;

		double actualCost;
		double heuristicCost;
};
/**
 * Return the coördinates of aVertex, combined with it's actual and heuristic costs. e.g. cout << aVertex
 */
std::ostream& operator<<(std::ostream& os, const Vertex & aVertex)
{
	return os << "(" << aVertex.x << "," << aVertex.y << "), " <<  aVertex.actualCost << " " << aVertex.heuristicCost;
}
/**
 * An edge is a connection between 2 points.
 * It has 2 vertices (each with their own x and y coördinates) as properties.
 *
 */
struct Edge
{
		//Initiate Edge, setting the 2 vertices
		Edge( const Vertex& aVertex1,const Vertex& aVertex2) :vertex1( aVertex1),vertex2( aVertex2){}

		//Check if
		bool isConnectedTo( const Vertex& aVertex) const
		{
			if(vertex1 == aVertex) return true;
			if(vertex2 == aVertex) return true;
			return false;
		}

		const Vertex& thisSide( const Vertex& aVertex) const
		{
			if (vertex1 == aVertex) return vertex1;
			if (vertex2 == aVertex) return vertex2;
			throw std::logic_error("thisSide: huh???");
		}

		const Vertex& otherSide( const Vertex& aVertex) const
		{
			if (vertex1 == aVertex) return vertex2;
			if (vertex2 == aVertex) return vertex1;
			throw std::logic_error("otherSide: huh???");
		}

		Vertex vertex1;
		Vertex vertex2;
};
/**
 *
 */

/**
 *
 */
typedef std::vector< Edge > Edges;
Edges edges;
/**
 * ClosedSet and VertexMap cannot use the same operator<() as OpenSet
 */
struct VertexCompare : public std::binary_function<Vertex, Vertex, bool>
{
	bool operator()( const Vertex& lhs, const Vertex& rhs) const
	{
		return lhs.getId() < rhs.getId();
	}
};
/**
 *
 */
typedef std::map< Vertex, Vertex, VertexCompare > VertexMap;
/**
 *
 */
typedef std::set< Vertex, VertexCompare > ClosedSet;
/**
 *
 */
typedef std::set< Vertex > OpenSet;
/**
 *
 */
double ActualCost( const Vertex& aFrom, const Vertex& aTo)
{
	return std::sqrt( (aFrom.x - aTo.x)*(aFrom.x - aTo.x) + (aFrom.y - aTo.y)*(aFrom.y - aTo.y) );
}
/**
 *
 */
double ActualCost( const Edge& anEdge)
{
	return ActualCost(anEdge.vertex1,anEdge.vertex2);
}
/**
 *
 */
double HeuristicCost( const Vertex& aFrom, const Vertex& aTo)
{
	return std::sqrt( (aFrom.x - aTo.x)*(aFrom.x - aTo.x) + (aFrom.y - aTo.y)*(aFrom.y - aTo.y) );
}
/**
 *
 */
std::vector<Vertex> ConstructPath( VertexMap& aFrom, const Vertex& aCurrentNode)
{
	VertexMap::iterator i = aFrom.find(aCurrentNode);
	if( i != aFrom.end() )
	{
		std::vector<Vertex> path = ConstructPath(aFrom,(*i).second);
		path.push_back(aCurrentNode);
		return path;
	}else
	{
		std::vector<Vertex> path;
		path.push_back(aCurrentNode);
		return path;
	}
}
/**
 *
 */
std::vector<Vertex> GetNeighbours(Vertex& aVertex)
{
	std::vector<Vertex> neighbours;
	for(const Edge& edge : edges)
	{
		if(edge.isConnectedTo(aVertex))
		{
			neighbours.push_back(edge.otherSide(aVertex));
		}
	}
	return neighbours;
}
/**
 * This is a straight forward C++ implementation of Wikipedia's A* pseudo code.
 * As proof of this the pseudo code is embedded in the source, just above the
 * C++ statements.
 *
 * See http://en.wikipedia.org/wiki/A*#Pseudocode for the complete pseudo code
 *
 * But also see the discussion page: it explains why the algorithm is only
 * correct if the cost function is monotone.This algorithm does not give the
 * correct solution in our case. The routes and hence costs differ whether we
 * go from (0,0) to (10,10) or vice versa:
 *
 * 0-0 -> 3-2 -> 4-5 -> 4-8 -> 6-9 -> 10-10, cost = 15
 *
 * 10-10 -> 10-7 -> 7-5 -> 6-3 -> 3-2 -> 0-0, cost = 14
 *
 * Use this piece of code as inspiration only.
 *
 */
std::vector<Vertex> WikiAStar( Vertex start, Vertex goal)
{
	ClosedSet closedSet;		// The set of nodes already evaluated.
	OpenSet openSet;			// The set of tentative nodes to be evaluated, initially containing the start node
	VertexMap predecessorMap;	// The map of navigated nodes.

	start.actualCost = 0.0; 												// Cost from start along best known path.
	start.heuristicCost = start.actualCost + HeuristicCost(start, goal);	// Estimated total cost from start to goal through y.
	openSet.insert(start);

	//while openset is not empty
	while(!openSet.empty())
	{
		// current := the node in openset having the lowest f_score[] value
		Vertex current = *openSet.begin();
		// if current = goal
		if(current == goal)
		{
			// return reconstruct_path(came_from, goal)
			return ConstructPath(predecessorMap,current);
		}
		// remove current from openset
		openSet.erase(current);
		// add current to closedset
		closedSet.insert(current);

		//for each neighbour in neighbour_nodes(current)
		std::vector< Vertex > neighbours = GetNeighbours(current);
		for(Vertex neighbour : neighbours)
		{
			// if neighbor in closedset continue (with next neighbour)
			if(closedSet.find(neighbour) != closedSet.end())
			{
				continue;
			}
			// tentative_g_score := g_score[current] + dist_between(current,neighbour)
			double calculatedActualNeighbourCost = current.actualCost + ActualCost(current,neighbour);

			// if neighbour not in openset or tentative_g_score < g_score[neighbour]
			if(openSet.find(neighbour) == openSet.end() || calculatedActualNeighbourCost < neighbour.actualCost)
			{
				// Here we deviate from the Wikipedia article, because of the map semantics:
				// we cannot change the object's key values once it in the map so we first
				// set the vales and then put it into the map.

				// g_score[neighbor] := tentative_g_score
				neighbour.actualCost = calculatedActualNeighbourCost;
				// f_score[neighbor] := g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
				neighbour.heuristicCost = neighbour.actualCost + HeuristicCost(neighbour, goal);

				// came_from[neighbor] := current
				std::pair<VertexMap::iterator,bool> result = predecessorMap.insert(std::make_pair(neighbour,current));

				// The following if-statement is not part of the pseudo code but a partial fix for a semantic difference
				// in the map of the pseudo code and the c++ std::map
				if(result.second==false)
					(*result.first).second = current;

				// if neighbor not in openset
				if(openSet.find(neighbour) == openSet.end())
				{
					// add neighbor to openset
					openSet.insert(neighbour);
				}
			}
		}
	}
	//return failure
	return std::vector<Vertex>();
}

int main( int /*argc*/, char **/*argv*/)
{

	edges.push_back(Edge( Vertex( 0,0), Vertex( 1,3)));
	edges.push_back(Edge( Vertex( 0,0), Vertex( 3,2)));
	edges.push_back(Edge( Vertex( 0,0), Vertex( 3,0)));
	edges.push_back(Edge( Vertex( 1,3), Vertex( 1,9)));
	edges.push_back(Edge( Vertex( 1,3), Vertex( 2,8)));
	edges.push_back(Edge( Vertex( 3,2), Vertex( 4,5)));
	edges.push_back(Edge( Vertex( 3,2), Vertex( 6,3)));
	edges.push_back(Edge( Vertex( 3,2), Vertex( 6,1)));
	edges.push_back(Edge( Vertex( 3,0), Vertex( 6,1)));
	edges.push_back(Edge( Vertex( 1,9), Vertex( 2,8)));
	edges.push_back(Edge( Vertex( 4,5), Vertex( 4,8)));
	edges.push_back(Edge( Vertex( 4,5), Vertex( 5,7)));
	edges.push_back(Edge( Vertex( 4,5), Vertex( 7,5)));
	edges.push_back(Edge( Vertex( 6,3), Vertex( 7,5)));
	edges.push_back(Edge( Vertex( 6,3), Vertex( 9,4)));
	edges.push_back(Edge( Vertex( 6,3), Vertex( 8,1)));
	edges.push_back(Edge( Vertex( 6,1), Vertex( 8,1)));
	edges.push_back(Edge( Vertex( 2,8), Vertex( 4,8)));
	edges.push_back(Edge( Vertex( 5,7), Vertex( 8,9)));
	edges.push_back(Edge( Vertex( 7,5), Vertex( 10,7)));
	edges.push_back(Edge( Vertex( 8,1), Vertex( 9,4)));
	edges.push_back(Edge( Vertex( 4,8), Vertex( 6,9)));
	edges.push_back(Edge( Vertex( 4,8), Vertex( 4,10)));
	edges.push_back(Edge( Vertex( 9,4), Vertex( 10,7)));
	edges.push_back(Edge( Vertex( 9,4), Vertex( 9,6)));
	edges.push_back(Edge( Vertex( 6,9), Vertex( 10,10)));
	edges.push_back(Edge( Vertex( 10,7), Vertex( 10,10)));

	{
		std::vector<Vertex> wikiPath = WikiAStar( Vertex( 8,9), Vertex( 10,10));

		std::vector<Vertex>::iterator i = wikiPath.begin();
		int cost = 0;
		while(i != wikiPath.end())
		{
			std::cout << (*i).getId();
			++i;
			if( i != wikiPath.end())
			{
				std::cout  << " -> ";
				cost += ActualCost( (*(i-1)),(*i));
			}else
			{
				std::cout << ", cost = " << cost <<std::endl;
			}
		}
	}

	{
		std::vector<Vertex> wikiPath = WikiAStar( Vertex( 10,10), Vertex( 8,9));

		std::vector<Vertex>::iterator i = wikiPath.begin();
		int cost = 0;
		while(i != wikiPath.end())
		{
			std::cout << (*i).getId();
			++i;
			if( i != wikiPath.end())
			{
				std::cout  << " -> ";
				cost += ActualCost( (*(i-1)),(*i));
			}else
			{
				std::cout << ", cost = " << cost <<std::endl;
			}
		}
	}

	return 0;
}
