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


void Graph::addVertex(char* node_start_name, 
					  char* node_end_name, 
					  unsigned char weight,
					  std::vector<std::function<void()>>& skills_vector)
{
	Node* node_start = findNode(node_start_name);
	Node* node_end = findNode(node_end_name);
	
	verteces_.push_back( Vertex(node_start, node_end, weight, skills_vector) );
	node_start->setVertex(&verteces_.back());
}


Node* Graph::findNode(char* name)
{
	for (auto& node : nodes_)
		if (node.getName() == name)
			return &node;
	std::cout << "Node not found" << std::endl;
	return NULL;
}


void Graph::showGraph()
{
	for (auto& node : nodes_){	
		std::cout << "Node \"" << node.getName() << "\" connected to:" << std::endl;
		for (auto& connected_node : node.getNodesConnected())
			std::cout << "   " << connected_node->getName() << std::endl;
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
	std::vector<std::function<void()>> solution;
	std::vector<Node*> open_list;
	Node* current_node;
	
	// Add the current node to the open list
	open_list.push_back(findNode(current_node_));
	current_node = open_list.front();
	
	// BFS
	while(!open_list.empty()){
		// Dequeue
		open_list.erase(open_list.begin());
		
		for(auto node_connected : current_node->getNodesConnected()){
			if (node_connected->getName() == node_end_name)
					
			
		}
	}
	
	for (auto& vertix : verteces_) {
		for (auto& skill : *vertix.getSkills() )
			solution.push_back(skill);
	
	//This is bad!
	return solution;
}
