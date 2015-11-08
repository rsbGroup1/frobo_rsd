#include <iostream>


#include "vertex.h"
#include "node.h"
#include "skills.h"

Vertex::Vertex(Node* node_start, Node* node_end, unsigned char weight, std::vector<std::function<void ()> >& skill_vector)
{
    node_start_ = node_start;
    node_end_ = node_end;
    weight_ = weight;
    skill_vector_ = skill_vector;
	
	node_start->setVertex(this);
	node_end->setVertex(this);
}

Vertex::~Vertex()
{
    // Nothing
}

std::vector<std::function<void ()> > * Vertex::getSkills()
{
    return &skill_vector_;
}

Node * Vertex::getNodeStart()
{
    return node_start_;
}

Node * Vertex::getNodeEnd()
{
    return node_end_;
}

unsigned char Vertex::getWeight()
{
    return weight_;
}
