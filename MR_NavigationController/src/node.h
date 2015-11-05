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
	void setConnection(Vertex& vertex){
		connections_.push_back(vertex);
	};
	
	/**
	 * Returns the name
	 * @param name_ the name of the node
	 */
	std::string getName(){
		return name_;
	}
	
	
private:
	char* name_;
	std::vector<Vertex> connections_;
};

#endif // NODE_H
