#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {
	
	BVHNode *root = new BVHNode();

	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB world_bbox = AABB(min, max);

	for (Object* obj : objs) {
		AABB bbox = obj->GetBoundingBox();
		world_bbox.extend(bbox);
		objects.push_back(obj);
	}
	world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
	world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
	root->setAABB(world_bbox);
	nodes.push_back(root);
	build_recursive(0, objects.size(), root); // -> root node takes all the 
}

/*
/ right_index, left_indexand split_index refer to the indices in the objects vector;
/ do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
/ node.index can have a index of objects vector or a index of nodes vector
*/
void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	// if only Threshold primitives in node, create leaf node
	if ((right_index - left_index) <= Threshold) {
		//node->makeLeaf(node->getIndex(), node->getNObjs());
		node->makeLeaf(left_index, right_index - left_index);
		return;
	}
	else {
		// find axis with max length (split)
		float split;
		int axis;
		split = node->getAABB().findSplit(axis); 

		// sort objects by axis
		Comparator cmp;
		cmp.dimension = axis;
		std::sort(objects.begin() + left_index, objects.begin() + right_index, cmp);
		
		// find split index to split objects
		int splitIndex = left_index;
		for (int i = left_index; i < right_index; i++) {
			Vector centroid = objects[i]->getCentroid();
			if (axis == 0) {
				splitIndex++;
				if (centroid.x > split) { break; }
			}
			else if (axis == 1) {
				splitIndex++;
				if (centroid.y > split) { break; }
			}
			else {
				splitIndex++;
				if (centroid.z > split) { break; }
			}
		}
		// Check if objects are all on right/left side, if so split in the middle
		if (splitIndex == left_index || splitIndex == right_index) { 
			float indexInterval = right_index - left_index;
			splitIndex = left_index + indexInterval / 2;
		}

		// Calculate the 2 new bounding boxes
		Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		AABB leftBB = AABB(min, max);
		for (int i = left_index; i < splitIndex; i++) {
			leftBB.extend(objects[i]->GetBoundingBox());
		}
		AABB rightBB = AABB(min, max);
		for (int i = splitIndex; i < right_index; i++) {
			rightBB.extend(objects[i]->GetBoundingBox());
		}

		// Create the 2 new BVH nodes
		BVHNode* nodeLeft = new BVHNode();
		nodeLeft->setAABB(leftBB);
		nodes.push_back(nodeLeft);
		node->makeNode(nodes.size()-1); // set current node child node (right node will be this index+1)

		BVHNode* nodeRight = new BVHNode();
		nodeRight->setAABB(rightBB);
		nodes.push_back(nodeRight);

		// recursively build rest of BVH
		build_recursive(left_index, splitIndex, nodeLeft);
		//printf("split to right : left: %d, right: %d\n", splitIndex, right_index);
		build_recursive(splitIndex, right_index, nodeRight);
	}
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
	bool hit = false;
	float tmin = FLT_MAX;  //contains the closest primitive intersection

	BVHNode* currentNode = nodes[0];

	// check if we hit world BB
	float t;
	if (!currentNode->getAABB().intercepts(ray, t)) {
		return false;
	}

	// init with root
	StackItem currentSI = StackItem(currentNode, t);

	while (true)
	{
		if (currentSI.t > tmin) { // skip node if we have a closer object intersection
			if (hit_stack.empty()) { break; }
			currentSI = hit_stack.top();
			hit_stack.pop();
			continue;
		} 

		if (currentSI.ptr->isLeaf())
		{
			// intersection test with each primitive in the leaf, get smallest primitive intersection
			for (int i = 0; i < currentSI.ptr->getNObjs(); i++)
			{
				Object* obj = objects[currentSI.ptr->getIndex() + i];
				float t;
				if (obj->intercepts(ray, t))
				{
					if (t < tmin)
					{
						tmin = t;
						*hit_obj = obj;
						hit = true;
					}
				}
			}

			if (hit_stack.empty()) { break; }

			// pop stack, go to next node (will be the closest one)
			currentSI = hit_stack.top();
			hit_stack.pop();
		}
		else
		{
			// check both children nodes intersection 
			BVHNode* leftNode = nodes[currentSI.ptr->getIndex()];
			BVHNode* rightNode = nodes[currentSI.ptr->getIndex()+1];
					
			float leftT, rightT;
			bool leftHit = leftNode->getAABB().intercepts(ray, leftT);
			bool rightHit = rightNode->getAABB().intercepts(ray, rightT);

			if (leftHit && rightHit)
			{
				// put furthest node in stack along with t, current node = closest 
				if (leftT > rightT)
				{
					StackItem sf = StackItem(leftNode, leftT);
					hit_stack.push(sf);

					currentSI = StackItem(rightNode, rightT);
				}
				else
				{
					StackItem sf = StackItem(rightNode, rightT);
					hit_stack.push(sf);

					currentSI = StackItem(leftNode, leftT);
				}
			}
			else if (leftHit)
			{
				currentSI = StackItem(leftNode, leftT);
			}
			else if (rightHit)
			{
				currentSI = StackItem(rightNode, rightT);
			}
			else // no hit
			{
				if (hit_stack.empty()) { break; }

				// pop stack, go to next node (will be the closest one)
				currentSI = hit_stack.top();
				hit_stack.pop();
			}
		}
	}

	
	hit_point = ray.RayPointFromDist(tmin);

	return hit;
}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
	float tmp;

	double length = ray.direction.length(); //distance between light and intersection point
	ray.direction.normalize();

	BVHNode* currentNode = nodes[0];

	// check if we hit world BB
	float t;
	if (!currentNode->getAABB().intercepts(ray, t)) {
		return false;
	}

	// init with root
	StackItem currentSI = StackItem(currentNode, t);

	while (true)
	{
		if (currentSI.ptr->isLeaf())
		{
			// intersection test with each primitive in the leaf
			for (int i = 0; i < currentSI.ptr->getNObjs(); i++)
			{
				Object* obj = objects[currentSI.ptr->getIndex() + i];
				float t;
				if (obj->intercepts(ray, t) && t < length) // if intercepted object is between hit point and light
				{
					return true;
				}
			}

			if (hit_stack.empty()) { break; }

			// pop stack, go to next node (will be the closest one)
			currentSI = hit_stack.top();
			hit_stack.pop();
		}
		else
		{
			// check both children nodes intersection 
			BVHNode* leftNode = nodes[currentSI.ptr->getIndex()];
			BVHNode* rightNode = nodes[currentSI.ptr->getIndex() + 1];

			float leftT, rightT;
			bool leftHit = leftNode->getAABB().intercepts(ray, leftT);
			bool rightHit = rightNode->getAABB().intercepts(ray, rightT);

			if (leftHit && rightHit)
			{
				// put furthest node in stack along with t, current node = closest 
				if (leftT > rightT)
				{
					StackItem sf = StackItem(leftNode, leftT);
					hit_stack.push(sf);

					currentSI = StackItem(rightNode, rightT);
				}
				else
				{
					StackItem sf = StackItem(rightNode, rightT);
					hit_stack.push(sf);

					currentSI = StackItem(leftNode, leftT);
				}
			}
			else if (leftHit)
			{
				currentSI = StackItem(leftNode, leftT);
			}
			else if (rightHit)
			{
				currentSI = StackItem(rightNode, rightT);
			}
			else // no hit
			{
				if (hit_stack.empty()) { break; }

				// pop stack, go to next node (will be the closest one)
				currentSI = hit_stack.top();
				hit_stack.pop();
			}
		}
	}

	return false;
}		
