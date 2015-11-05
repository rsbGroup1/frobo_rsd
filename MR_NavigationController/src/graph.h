#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <functional>

class Skills;
class Node;
class Vertex;

class Graph
{
public:
	Graph();
	
	~Graph();
	
	void createNode(char* name);
	
	void createConnection(char* node_start_name, char* node_end_name, 
						  unsigned char weight, std::vector<std::function<void()>>& skills_vector);
	
	Node* findNode(char* name);
	
	void showGraph();
	
private:
	std::vector<Node> nodes_;
	std::vector<Vertex> vertex_;
	
};

#endif // GRAPH_H
