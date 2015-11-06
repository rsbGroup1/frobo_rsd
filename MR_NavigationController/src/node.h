#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <string>
#include <vector>

#include "vertex.h"

class Node
{
	
public:
	/**
	 * Initialize a node with a name
	 */
	Node(char* name)
	{
		name_ = name;
	};
	
	/**
	 * Destructs the node
	 */
	~Node(){
		// Nothing
	};
	
	/**
	 * Add a vertex to the node
	 * @param vertex the vertex to add
	 */
	void setVertex(Vertex* vertex){
		connections_.push_back(vertex);
		nodes_connected_.push_back(vertex->getNodeEnd());
	};
	
	/**
	 * Returns the name
	 * @return name_ the name of the node
	 */
	std::string getName(){
		return name_;
	}
	
	/**
	 * Returns the vector of connected nodes
	 * @return nodes_connected_ the nodes connected
	 */
	std::vector<Node*> getConnectionsName(){
		return nodes_connected_;
	}
	
	
private:
	char* name_;
	std::vector<Vertex*> connections_;
	std::vector<Node*> nodes_connected_;
};

#endif // NODE_H
