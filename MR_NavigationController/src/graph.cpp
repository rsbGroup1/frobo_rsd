#include "graph.h"

#include <iostream>

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
    for (auto& node : nodes_) {
        std::cout << "Node \"" << node.getName() << "\""<< std::endl;
        if (node.getParentsNodes().empty() && node.getChildrenNodes().empty()) {
            std::cout << "    NOT connected" << std::endl;
        }
        if (!node.getChildrenNodes().empty()) {
            std::cout << "    Parent of: " << std::endl;
            for (auto& connected_node : node.getChildrenNodes())
                std::cout << "       " << connected_node->getName() << std::endl;
        }
        if (!node.getParentsNodes().empty()) {
            std::cout << "    Children of: " << std::endl;
            for (auto& connected_node : node.getParentsNodes())
                std::cout << "       " << connected_node->getName() << std::endl;
        }
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

std::vector<std::function<void()>> Graph::bfs(const char* node_end_name, int number_limit)
{
    std::cout << "BFS Search from \"" << current_node_ <<"\" to \"" << node_end_name << "\"" << std::endl;

    std::vector<std::function<void()>> solution;
    std::vector<Node*> open_list;
    std::vector<Node*> closed_list;
    Node* current_node;
    unsigned char counter = 0;


    // Clean the parents
    for (auto& node_to_clean : nodes_)
        node_to_clean.setParent(NULL);

    // Add the current node to the open list
    open_list.push_back(findNode(current_node_));

    // BFS
	while(!open_list.empty() && ++counter!=number_limit && solution.empty()) {
        // Choose a new node
        current_node = open_list.front(); // Updates the current node
        closed_list.push_back(current_node); // Put it in the closed list
        open_list.erase(open_list.begin()); // Dequeue in the open list
        // Check if we have found the solution
        if (current_node->getName() == node_end_name) {
            // Until the original node is not reached
            do {
                for (auto& vertex : verteces_) {
                    if (vertex.getNodeEnd() == current_node &&
                            vertex.getNodeStart() == current_node->getParent())
                    {
                        for (unsigned char i=vertex.getSkills()->size(); i>0; i--){
                            solution.insert(solution.begin(), vertex.getSkills()->at(i-1));
						}
                    }
                }
                current_node = current_node->getParent();
            } while (current_node->getParent() != NULL); // The parent of the first node is NULL
        } else {
            // Add children
            for(auto node_connected : current_node->getChildrenNodes()) {
                // Check if it exists in the closed list
                for (auto& node_in_closed_list : closed_list) {
                    if (node_in_closed_list != node_connected) {
                        // If it is not, set the parent and put it in the open list
                        node_connected->setParent(current_node);
                        open_list.push_back(node_connected);
                    }
                }
            }
        }
    }

    // Solution not found message
    if (solution.empty())
        std::cout << "Solution NOT found" << std::endl;
	if (counter == number_limit)
		std::cout << "Search limit reached" << std::endl;

    return solution;
}
