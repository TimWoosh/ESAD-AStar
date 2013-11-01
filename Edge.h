/*
 * Edge.h
 *
 *  Created on: 1 nov. 2013
 *      Author: TWostemeier
 */

#ifndef EDGE_H_
#define EDGE_H_

#include "Vertex.h"

class Edge {
	public:
		/**
		 *
		 */
		Edge();
		/**
		 *
		 * @param aVertex1
		 * @param aVertex2
		 */
		Edge(const Vertex& aVertex1, const Vertex& aVertex2);
		/**
		 *
		 * @param anEdge
		 */
		Edge(const Edge& anEdge);
		/**
		 *
		 */
		~Edge();
		/**
		 *
		 * @param aVertex
		 * @return
		 */
		bool isConnectedTo(const Vertex& aVertex) const;
		/**
		 *
		 * @param aVertex
		 * @return
		 */
		const Vertex& thisSide(const Vertex& aVertex) const;
		/**
		 *
		 * @param aVertex
		 * @return
		 */
		const Vertex& otherSide(const Vertex& aVertex) const;
		/**
		 *
		 * @param anEdge
		 * @return
		 */
		Edge& operator =(const Edge& anEdge);

	private:
		/**
		 *
		 */
		Vertex vertex1;
		/**
		 *
		 */
		Vertex vertex2;
};



#endif /* EDGE_H_ */
