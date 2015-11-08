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
	if (vertex->getNodeEnd()->getName() == this->getName())
		parents_.push_back(vertex->getNodeStart());
	if (vertex->getNodeStart()->getName() == this->getName())
		childrens_.push_back(vertex->getNodeEnd());
}


std::string Node::getName()
{
    return name_;
}


std::vector<Node *> Node::getChildrenNodes()
{
    return childrens_;
}


std::vector<Node *> Node::getParentsNodes()
{
    return parents_;
}


Node * Node::getParent()
{
    return parent_;
}

void Node::setParent(Node* parent)
{
    parent_ = parent;
}
