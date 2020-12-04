// For KD-Tree implementation
// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT point_, int setId)
	:	point(point_), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}
	
	void insertHelper(Node<PointT>** node, uint depth, PointT point, int id){
		// Check if node is empty or not
		if((*node)==NULL){
			*node = new Node<PointT>(point,id);
		} else {
			// Decide to check whether x or y
			auto depth_idx = depth % 2;
            if(depth_idx){
            // IF depth_idx == 0 check x else y
                if (point.x < (*node)->point.x)
				    insertHelper(&((*node)->left), depth+1, point,id);	
			    else
				    insertHelper(&((*node)->right), depth+1, point,id);    
            } else {
                if (point.y < (*node)->point.y)
				    insertHelper(&((*node)->left), depth+1, point,id);	
			    else
				    insertHelper(&((*node)->right), depth+1, point,id);    
            }	
		}
	}
    
	void insert(PointT point, int id)
	{
		// Fill new point into the tree
		auto init_depth = 0;
		insertHelper(&root,init_depth, point, id);	
	}

	bool IsPointInPolygon(PointT target, float distanceTol, PointT point){
		if(abs(point.x-target.x)<=distanceTol && abs(point.y-target.y)<=distanceTol)
			return true;   
		return false;
	}
    
	bool IsPointInBorders(PointT target, float distanceTol, PointT point){
		if(abs(point.x-target.x)<=distanceTol || abs(point.y-target.y)<=distanceTol)
			return true;   
		return false;
	}
    
	void searchHelper(Node<PointT>** node, uint depth, PointT target, float distanceTol, std::vector<int>* indices){
		// Check if node is empty or not 
		if((*node)!=NULL){
			// Check if the node is inside the rectangle
			if(IsPointInPolygon(target, distanceTol, (*node)->point) == true){
				indices->push_back((*node)->id);	
			}
			
			auto depth_idx = depth % 2;
            if(depth_idx){
                // IF depth_idx == 0 check x else y
    			if((target.x-distanceTol)<(*node)->point.x){
    				searchHelper(&((*node)->left), depth+1, target, distanceTol, indices);
    			}
    			if((target.x+distanceTol)>(*node)->point.x){
    				searchHelper(&((*node)->right), depth+1, target, distanceTol, indices);
    			}
            } else {
    			if((target.y-distanceTol)<(*node)->point.y){
    				searchHelper(&((*node)->left), depth+1, target, distanceTol, indices);
    			}
    			if((target.y+distanceTol)>(*node)->point.y){
    				searchHelper(&((*node)->right), depth+1, target, distanceTol, indices);
    			} 
            }	
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		auto init_depth = 0;
		searchHelper(&root, init_depth, target, distanceTol, &ids);
		return ids;
	}
};