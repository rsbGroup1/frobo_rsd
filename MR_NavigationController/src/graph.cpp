#include "graph.h"

#include <iostream>

#include "std_msgs/String.h"

Graph::Graph (ros::Publisher* pub_current_node)
{
    pub_current_node_ = pub_current_node;
    current_node_ = "line_start"; // Default position
}

Graph::~Graph()
{
    // Nothing
}


void Graph::addNode (std::string name)
{
    nodes_.push_back (Node (name));
}


void Graph::addVertex (std::string node_start_name,
                       std::string node_end_name,
                       unsigned char weight,
                       std::vector<std::function<void() >>& skills_vector)
{
    Node* node_start = findNode (node_start_name);
    Node* node_end = findNode (node_end_name);

    verteces_.push_back (Vertex (node_start, node_end, weight, skills_vector));
}


Node* Graph::findNode (std::string name)
{
    for (auto & node : nodes_)
        if (node.getName() == name)
            return &node;

    std::cout << "Node not found" << std::endl;
    return NULL;
}


void Graph::showGraph()
{
    for (auto & node : nodes_)
    {
        std::cout << "Node \"" << node.getName() << "\"" << std::endl;

        if (node.getParentsNodes().empty() && node.getChildrenNodes().empty())
        {
            std::cout << "    NOT connected" << std::endl;
        }

        if (!node.getChildrenNodes().empty())
        {
            std::cout << "    Parent of: " << std::endl;

            for (auto & connected_node : node.getChildrenNodes())
                std::cout << "       " << connected_node->getName() << std::endl;
        }

        if (!node.getParentsNodes().empty())
        {
            std::cout << "    Children of: " << std::endl;

            for (auto & connected_node : node.getParentsNodes())
                std::cout << "       " << connected_node->getName() << std::endl;
        }
    }
}


std::string Graph::getCurrentNode()
{
    return current_node_;
}


void Graph::setCurrentNode (std::string name_of_current_node)
{
    current_node_ = name_of_current_node;
    std::cout << "Current node: " << current_node_ << std::endl;
    std_msgs::String msg;
    msg.data = (std::string) (name_of_current_node);
    pub_current_node_->publish (msg);
}

std::vector< Node > Graph::getNodes()
{
    return nodes_;
}

std::vector<std::function<void() >> Graph::bfs (const std::string node_end_name, int number_limit)
{
    std::cout << "BFS Search from \"" << current_node_ << "\" to \"" << node_end_name << "\"" << std::endl;

    std::vector<std::function<void() >> solution;
    std::vector<Node*> open_list;
    std::vector<Node*> closed_list;
    Node* current_node;
    int counter;

    // Initialize the BFS
    open_list.clear();
    closed_list.clear();
    solution.clear();
    current_node = NULL;
    counter = 0;

    // Check if it is a illegal movement
    if (node_end_name == current_node_)
    {
        ROS_INFO ("You are already there!");
        return solution;
    }

    // Add the current node to the open list
    Node* first_node = new Node(current_node_);
	first_node->setParent(NULL);
	open_list.push_back (first_node);

    // BFS
    while (!open_list.empty() && ++counter != number_limit && solution.empty())
    {
        // Choose a new node
		try 
		{
        current_node = open_list.front(); // Updates the current node
        closed_list.push_back (current_node); // Put it in the closed list
        open_list.erase (open_list.begin()); // Dequeue in the open list
		}
		catch (const std::exception e) 
		{
			std::cout << "Error selecting node: " << e.what() << std::endl;
		}
		

        // Check if we have found the solution
        if (current_node->getName() == node_end_name)
        {
			try {
            // Until the original node is not reached
			std::cout << "    ";
				do
				{
					for (auto & vertex : verteces_)
					{
						if (vertex.getNodeEnd()->getName() == current_node->getName() &&
								vertex.getNodeStart()->getName() == current_node->getParent()->getName())
						{
							std::cout << vertex.getNodeEnd()->getName() << " << ";
							for (auto i = vertex.getSkills()->size(); i > 0; i--)
								solution.insert (solution.begin(), vertex.getSkills()->at (i - 1));
							
						}
					}
					current_node = current_node->getParent();
				}
				while (current_node->getParent() != NULL);   // The parent of the first node is "no_parent"
				std::cout << current_node->getName() << std::endl;
			}
			catch (const std::exception& e)
			{
				std::cout << "Error constructing path: " << e.what() << std::endl;
			}
        }
        else
        {
            // Add children
			try 
			{
				for (auto node_connected : findNode(current_node->getName())->getChildrenNodes())
				{
					bool new_node = true;

					// Check if it exists in the closed list
					for (auto & node_in_closed_list : closed_list) 
					{
						if (node_in_closed_list->getName() == node_connected->getName()
							&& node_in_closed_list->getParent() != NULL
							&& node_in_closed_list->getParent()->getName() == current_node->getName())
						{
							new_node = false;
						}
					}

					//std::cout << "Closed node: " << node_connected->getName() << std::endl;
					// If it is not, set the parent and put it in the open list
					if (new_node == true)
					{
						Node* node_to_add = new Node(node_connected->getName());
						node_to_add->setParent (current_node);
						open_list.push_back (node_to_add);
					}
				}
				/*
				std::cout << "Open list: ";
				for (auto node : open_list)
					std::cout << node->getName() << ",";
				std::cout << std::endl;
				*/
			}
			catch (const std::exception e)
			{
				std::cout << "Error inflating node: " << e.what() << std::endl;
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
