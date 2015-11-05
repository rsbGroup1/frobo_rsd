#include "graph.h"

#include "node.h"
#include "skills.h"
#include "vertex.h"


Graph::Graph()
{
}

Graph::~Graph()
{
}

void Graph::createNode(char* name)
{
	nodes_.push_back(Node(name));
}

void Graph::createConnection(char* node_start_name, char* node_end_name, unsigned char weight,std::vector<std::function<void()>>& skills_vector)
{
	Node* node_start = findNode(node_start_name);
	Node* node_end = findNode(node_end_name);
	
	vertex_.push_back( Vertex(node_start, node_end, weight, skills_vector) );
	
}

Node* Graph::findNode(char* name)
{
	for (auto node : nodes_)
		if (node.getName() == name)
			return &node;
	std::cout << "Node not found" << std::endl;
	return NULL;
}


void Graph::showGraph()
{
}

