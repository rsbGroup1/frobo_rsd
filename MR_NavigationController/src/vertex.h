#ifndef VERTEX_H
#define VERTEX_H

#include <iostream>
#include <string>
#include <vector>
#include <functional>

#include "node.h"
#include "skills.h"

class Vertex
{
public:
	/**
	 * Create the vertex
	 * @param node_start node from which the vertex starts
	 * @param node_end node into the vertex ends
	 * @param weight the weight of the join
	 * @param skill_vector the skills to do from going to one node to the other
	 */
    Vertex(Node* node_start, Node* node_end, unsigned char weight,
		   std::vector<std::function<void()>>& skill_vector)
    {
        node_start_ = node_start;
        node_end_ = node_end;
        weight_ = weight;
        skill_vector_ = skill_vector;
    }
    
    /**
	 * Destructor
	 */
    ~Vertex(){
		// Nothing
	};
	
	/**
	 * Returns the vector of skills
	 */
	std::vector<std::function<void()>>* getSkills(){
		return &skill_vector_;
	}
	
	/**
	 * Returns the start node
	 */
	Node* getNodeStart(){
		return node_start_;
	}
	
	/**
	 * Returns the end node
	 */
	Node* getNodeEnd(){
		return node_end_;
	}
	
	/**
	 * Returns the weight
	 */
	unsigned char getWeight(){
		return weight_;
	}

private:
    Node* node_start_;
    Node* node_end_;
	std::vector<std::function<void()>> skill_vector_;
    unsigned char weight_;
};

#endif // VERTEX_H
