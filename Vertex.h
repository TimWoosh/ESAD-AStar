/*
 * Vertex.h
 *
 *  Created on: 1 nov. 2013
 *      Author: TWostemeier
 */

#ifndef VERTEX_H_
#define VERTEX_H_


class Vertex {
	public:
		/**
		 *
		 */
		Vertex();
		/**
		 *
		 * @param anX
		 * @param anY
		 */
		Vertex(int anX, int anY);
		/**
		 *
		 * @param aVertex
		 */
		Vertex(const Vertex& aVertex);
		/**
		 *
		 */
		~Vertex();
		/**
		 *
		 * @return
		 */
		std::string getId() const;
		/**
		 *
		 * @param aVertex
		 * @return
		 */
		Vertex& operator =(const Vertex& aVertex);
		/**
		 *
		 * @param aVertex
		 * @return
		 */
		bool operator<(const Vertex& aVertex) const;
		/**
		 *
		 * @param aVertex
		 * @return
		 */
		bool operator==(const Vertex& aVertex) const;


	private:
		/**
		 *
		 */
		int x;
		/**
		 *
		 */
		int y;
		/**
		 *
		 */
		double actualCost;
		/**
		 *
		 */
		double heuristicCost;
};


#endif /* VERTEX_H_ */
