#ifndef VERTEX_H
#define VERTEX_H

#include <vector>
#include <functional>

class Node;
class Skills;

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
		   std::vector<std::function<void()>>& skill_vector);
    
    /**
	 * Destructor
	 */
    ~Vertex();
	
	/**
	 * Returns the vector of skills
	 */
	std::vector<std::function<void()>>* getSkills();
	
	/**
	 * Returns the start node
	 */
	Node* getNodeStart();
	
	/**
	 * Returns the end node
	 */
	Node* getNodeEnd();
	
	/**
	 * Returns the weight
	 */
	unsigned char getWeight();

private:
    Node* node_start_;
    Node* node_end_;
	std::vector<std::function<void()>> skill_vector_;
    unsigned char weight_;
};

#endif // VERTEX_H
