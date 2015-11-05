#include "graph.h"

#include "node.h"
#include "skills.h"
#include "vertex.h"


Graph::Graph()
{
}

Graph::~Graph()
{
	// Nothing
}


void Graph::addNode(char* name)
{
	nodes_.push_back(Node(name));
}


void Graph::addVertex(char* node_start_name, char* node_end_name, 
					  unsigned char weight,
					  std::vector<std::function<void()>>& skills_vector)
{
	Node* node_start = findNode(node_start_name);
	Node* node_end = findNode(node_end_name);
	
	verteces_.push_back( Vertex(node_start, node_end, weight, skills_vector) );
	
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
	for (auto node : nodes_){	
		std::cout << "Node \"" << node.getName() << "\" connected to:" << std::endl;
		for (auto connected_node : node.getConnectionsName())
			std::cout << "   " << connected_node.getName() << std::endl;
	}
}


char * Graph::getCurrentNode()
{
	return current_node_;
}


void Graph::setCurrentNode(char* name_of_current_node)
{
	current_node_ = name_of_current_node;
}

std::vector<std::function<void()>> Graph::bfs(const char* node_end_name)
{
	//This is bad!
	return verteces_[0].getSkills();
}
