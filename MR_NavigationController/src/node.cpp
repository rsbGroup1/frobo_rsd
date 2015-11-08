#include <iostream>

#include "node.h"
#include "vertex.h"

Node::Node(std::string name)
{
    name_ = name;
    parent_ = NULL;
}

Node::~Node()
{
    // Nothing
}


void Node::setVertex(Vertex* vertex)
{
    verteces_.push_back(vertex);
	if (vertex->getNodeStart()->getName() == this->getName())
		children_of_.push_back(vertex->getNodeEnd());
	else if (vertex->getNodeEnd()->getName() == this->getName())
		parent_of_.push_back(vertex->getNodeEnd());
}


std::string Node::getName()
{
    return name_;
}


std::vector<Node *> Node::getChildrenNodes()
{
    return children_of_;
}


std::vector<Node *> Node::getParentsNodes()
{
    return parent_of_;
}


Node * Node::getParent()
{
    return parent_;
}

void Node::setParent(Node* parent)
{
    parent_ = parent;
}

Vertex * Node::getVertexFrom(Node* node)
{
    for (auto& vertex : verteces_){
		std::cout <<  verteces_[0]->getWeight() << std::endl;
		std::cout << vertex->getNodeStart()->getName() << std::endl;
		if (vertex->getNodeEnd()->getName() == this->getName() &&
			vertex->getNodeStart()->getName() == node->getName())
			return vertex;
	}
    std::cout << "Not vertex found" << std::endl;
    return NULL;
}
