/*
 * Edge.cpp
 *
 *  Created on: 1 nov. 2013
 *      Author: TWostemeier
 */

#include <iostream>
#include <stdexcept>
#include "Edge.h"
#include "Vertex.h"

Edge::Edge()
	: vertex1(Vertex()),
	  vertex2(Vertex()){};

Edge::Edge(const Vertex& aVertex1, const Vertex& aVertex2)
	: vertex1(aVertex1),
	  vertex2(aVertex2){};

Edge::Edge(const Edge& anEdge)
	: vertex1(anEdge.vertex1),
	  vertex2(anEdge.vertex2){};

Edge::~Edge()
{
	std::cout << "Destructing edge from " << vertex1.getId() << " to " << vertex2.getId() << std::endl;
}

bool Edge::isConnectedTo(const Vertex& aVertex) const
{
	if(this->vertex1 == aVertex) return true;
	if(this->vertex2 == aVertex) return true;
	return false;
}

const Vertex& Edge::thisSide(const Vertex& aVertex) const
{
	if (this->vertex1 == aVertex) return this->vertex1;
	if (this->vertex2 == aVertex) return this->vertex2;
	throw std::logic_error("thisSide: huh???");
}

const Vertex& Edge::otherSide(const Vertex& aVertex) const
{
	if (this->vertex1 == aVertex) return this->vertex2;
	if (this->vertex2 == aVertex) return this->vertex1;
	throw std::logic_error("otherSide: huh???");
}

Edge& Edge::operator =(const Edge& anEdge)
{
	this->vertex1 = anEdge.vertex1;
	this->vertex2 = anEdge.vertex2;
	return *this;
}
