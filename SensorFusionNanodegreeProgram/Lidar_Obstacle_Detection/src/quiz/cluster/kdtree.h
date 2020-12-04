/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
	void insertHelper(Node** node, uint depth, std::vector<float> point, int id){
		// Check if node is empty or not
		if((*node)==NULL){
			*node = new Node(point,id);
		} else {
			// Decide to check whether x or y
			auto depth_idx = depth % 2;
			if (point[depth_idx] < (*node)->point[depth_idx])
				insertHelper(&((*node)->left), depth+1, point,id);	
			else
				insertHelper(&((*node)->right), depth+1, point,id);	
		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		auto init_depth = 0;
		insertHelper(&root,init_depth, point, id);	
	}

	bool IsPointInPolygon(std::vector<float> target, float distanceTol, std::vector<float> point){

		if(abs(point[0]-target[0])<=distanceTol && abs(point[1]-target[1])<=distanceTol)
			return true;   
		return false;
	}
	bool IsPointInBorders(std::vector<float> target, float distanceTol, std::vector<float> point){

		if(abs(point[0]-target[0])<=distanceTol || abs(point[1]-target[1])<=distanceTol)
			return true;   
		return false;
	}
	void searchHelper(Node** node, uint depth, std::vector<float> target, float distanceTol, std::vector<int>* indices){
		// Check if node is empty or not 
		if((*node)!=NULL){
			// Check if the node is inside the rectangle
			if(IsPointInPolygon(target, distanceTol, (*node)->point) == true){
				indices->push_back((*node)->id);	
			}
			
			auto depth_idx = depth % 2;
			
			if((target[depth_idx]-distanceTol)<(*node)->point[depth_idx]){
				searchHelper(&((*node)->left), depth+1, target, distanceTol, indices);
			}
			if((target[depth_idx]+distanceTol)>(*node)->point[depth_idx]){
				searchHelper(&((*node)->right), depth+1, target, distanceTol, indices);
			}
			
			
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		
		auto init_depth = 0;
		searchHelper(&root, init_depth, target, distanceTol, &ids);
		return ids;
	}
	
	

};




