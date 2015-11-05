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
    Vertex(Node* node_start, Node* node_end, unsigned char weight,
		   std::vector<std::function<void()>>& skill_vector)
    {
        node_start_ = node_start;
        node_end_ = node_end;
        weight_ = weight;
        skill_vector_ = &skill_vector;
    }
    
    ~Vertex(){
		// Nothing
	};
	
	std::vector<std::function<void()>>* getSkills(){
		return skill_vector_;
	}

private:
    Node* node_start_;
    Node* node_end_;
	std::vector<std::function<void()>>* skill_vector_;
    unsigned char weight_;
};

#endif // VERTEX_H
