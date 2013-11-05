/*
 * Vertex.cpp
 *
 *  Created on: 1 nov. 2013
 *      Author: TWostemeier
 */

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <typeinfo>
#include "Vertex.h"

Vertex::Vertex()
	:x(0),
	 y(0),
	 actualCost(0.0),
	 heuristicCost(0.0){};

Vertex::Vertex(int anX, int anY)
	:x(anX),
	y(anY),
	actualCost(0.0),
	heuristicCost(0.0){};

Vertex::Vertex(const Vertex& aVertex)
	: x(aVertex.x),
	  y(aVertex.y),
	  actualCost(aVertex.actualCost),
	  heuristicCost(aVertex.heuristicCost){};

Vertex::~Vertex()
{
	std::cout << "Destructing Vertex " << this->getId() << std::endl;
}

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

//std::string Vertex::getId() const
//{
//	return anyToString(this->x) + "-" + anyToString(y);
//}

Vertex& Vertex::operator =(const Vertex& aVertex)
{
	this->x = aVertex.x;
	this->y = aVertex.y;
	this->actualCost = aVertex.actualCost;
	this->heuristicCost = aVertex.heuristicCost;
	return *this;
}

bool Vertex::operator<(const Vertex& aVertex) const
{
	if(this->heuristicCost < aVertex.heuristicCost) return true;
	//if(heuristicCost == aVertex.heuristicCost) return actualCost > aVertex.actualCost; // less uncertainty
	return false;
}

bool Vertex::operator==(const Vertex& aVertex) const
{
	return this->x == aVertex.x && this->y == aVertex.y;
}
