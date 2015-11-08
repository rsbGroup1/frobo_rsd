#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <functional>

#include "ros/ros.h"

class Node;
class Vertex;

class Graph
{
public:
	/**
	 * Initialize the graph given a publisher for the current node
	 */
	Graph(ros::Publisher * pub_current_node);
	
	~Graph();
	
	/**
	 * Adds a node to the graph given a name
	 * @param name the name of the node. This is used as an identifier
	 */
	void addNode(char* name);
	
	/**
	 * Adds a vertex to the node given two nodes and adds a weight
	 * for the connection and also the vector of skills that are 
	 * necessary to move from one state to the other.
	 * @param node_start_name the name of the node to start from
	 * @param node_end_name the name of the node to end
	 * @param weight the weight of the connection. This is used to 
	 * calculate the optimal path given two nodes
	 * @param skills_vector the vector of skills that join both nodes
	 */
	void addVertex(char* node_start_name, char* node_end_name, 
						  unsigned char weight, 
				std::vector<std::function<void()>>& skills_vector);
	
	/**
	 * Given a name, return the pointer to the node with
	 * that name
	 * @param name the name of the node to search
	 */
	Node* findNode(char* name);
	
	/**
	 * Shows a representation of the graph
	 */
	void showGraph();
	
	/**
	 * Searchs the path to the desired node and return the vector
	 * of skills to achieve it
	 * @param node_end_name the node to search
	 */
	std::vector<std::function<void()>> bfs(const char* node_end_name, int number_limit);
	
	/**
	 * Sets the current node in which the robot is located and publish it
	 * @param name_of_current_node the name of the current node
	 */
	void setCurrentNode(char* name_of_current_node);
	
	/**
	 * Gets the current node in which the robot is located
	 */
	char* getCurrentNode();
	
private:
	std::vector<Node> nodes_;
	std::vector<Vertex> verteces_;
	char* current_node_;
	ros::Publisher * pub_current_node_;
};

#endif // GRAPH_H
