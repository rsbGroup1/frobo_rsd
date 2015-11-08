#ifndef NODE_H
#define NODE_H

#include <string>
#include <vector>

class Vertex;

class Node
{
public:
	/**
	 * Initialize a node with a name
	 */
	Node(std::string name);
	
	/**
	 * Destructs the node
	 */
	~Node();
	
	/**
	 * Add a vertex to the node
	 * @param vertex the vertex to add
	 */
	void setVertex(Vertex* vertex);
	
	/**
	 * Returns the name
	 * @return name_ the name of the node
	 */
	std::string getName();
	
	/**
	 * Return the parent of the node
	 */
	Node* getParent();
	
	/**
	 * Set the parent of the node
	 */
	void setParent(Node* parent);
	
	/**
	 * Returns a vector with all the nodes that are the children of
	 * the node
	 */
	std::vector<Node*> getChildrenNodes();
	
	/**
	 * Returns a vector with all the nodes that are the parent
	 * of node
	 */
	std::vector<Node*> getParentsNodes();
	
	
private:
	std::string name_;
	std::vector<Vertex*> verteces_;
	std::vector<Node*> parents_;
	std::vector<Node*> childrens_;
	Node* parent_;
};

#endif // NODE_H
