#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
    
  
  // Create bounding box
    BBox bbox;
    for (auto p = start; p != end; p++) {
        BBox bb = (*p)->get_bbox();
        bbox.expand(bb);
    }

    // Create new node object
    BVHNode* node = new BVHNode(bbox);

    // Number of primitives
    int num_primitives = (end - start);

    // Base Case - Create a leaf node
    if (num_primitives <= max_leaf_size) {
        node->l = NULL;
        node->r = NULL;
        node->start = start;
        node->end = end;
    }
    else {
        Vector3D mean(0,0,0);

        for (auto p = start; p != end; p++) {
            mean += (*p)->get_bbox().centroid();
        }
        mean /= (float)num_primitives;

              
        int min_split_axis = 0;            // Splitting axis with bounding boxes having smallest surface area
        float min_BBox_heuristic = INFINITY;   // Heuristic for choosing which cartesian axis to perform mean split along
        
        
        // ---------------- ASSESSING SPLIT AXIS HEURISTIC ---------------------------
        for (int axis = 0; axis < 3; axis++) {

            // Create and populate left and right primitive vectors
            std::vector<Primitive*> left_primitives;
            std::vector<Primitive*> right_primitives;
            
            for (auto p = start; p != end; p++) {
                if ((*p)->get_bbox().centroid()[axis] <= mean[axis]) {
                    left_primitives.push_back(*p);
                }
                else {
                    right_primitives.push_back(*p);
                }
            }

            BBox left_bbox;
            BBox right_bbox;
            // build the child bounding boxes
            for (int i = 0; i < left_primitives.size();i++) {
                BBox bb = left_primitives[i]->get_bbox();
                left_bbox.expand(bb);
            }
            for (int j = 0; j < right_primitives.size(); j++) {
                BBox bb = right_primitives[j]->get_bbox();
                right_bbox.expand(bb);
            }
            

            // compute the BBox heuristic for the current splitting axis
            Vector3D exL = left_bbox.extent;
            Vector3D exR = right_bbox.extent;
            float heuristic = left_primitives.size()*(exL.x * exL.y + exL.y * exL.z + exL.z * exL.x) 
                            + right_primitives.size()*(exR.x * exR.y + exR.y * exR.z + exR.z * exR.x);
            if (heuristic < min_BBox_heuristic) {
                min_BBox_heuristic = heuristic;
                min_split_axis = axis;
            }
        }
        // ------------------- END HEURISTIC ASSESSMENT -------------------------
        

        // Update start/end pointers and perform the recursive step
        std::vector<Primitive*> left_primitives;
        std::vector<Primitive*> right_primitives;

        for (auto p = start; p != end; p++) {
            //if ((*p)->get_bbox().centroid().z <= mean.z) {
            if ((*p)->get_bbox().centroid()[min_split_axis] <= mean[min_split_axis]) {
                left_primitives.push_back(*p);
            }
            else {
                right_primitives.push_back(*p);
            }
        }
        auto center_left = start;
        auto center_right = start;
        for (int i = 0; i < num_primitives; i++) {
            if (i < left_primitives.size()) {
                *center_left = left_primitives[i];
                center_left++;
            }
            else {
                *center_right = right_primitives[i - left_primitives.size()];
            }
            center_right++;
        }
        node->l = construct_bvh(start, center_left, max_leaf_size);
        node->r = construct_bvh(center_left, end, max_leaf_size);

        
        /*
        // Sort primitives along x-axis using lambda function comparator
        sort(start, end, [](const Primitive& p1, const Primitive& p2)
            {
                return p1.get_bbox().centroid().x < p2.get_bbox().centroid().x;
            });
        */

    }

  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
    
    if (node->bb.intersect(ray, ray.min_t, ray.max_t)) {
        if (node->isLeaf()) {
            for (auto p = node->start; p != node->end; p++) {
                if ((*p)->has_intersection(ray)) {
                    return true;
                }
            }
        }
        else {
            return has_intersection(ray, node->l) || has_intersection(ray, node->r);
        }
    }
    else {
        return false;
    }
    /*
    
    // prev code
    for (auto p : primitives) {
    total_isects++;
    if (p->has_intersection(ray))
        return true;
    }
    return false;
    */


}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
    //std::cout << "isLeaf" << std::endl; //PRINT CMD
    // check if ray interesects with current node's Bbox
    
    
    total_isects++; // number of interesection tests for the ray
    if (node->bb.intersect(ray, ray.min_t, ray.max_t)) {
        
        // Node is a leaf
        if (node->isLeaf()) {
            
            bool hit = false;
            for (auto p = node->start; p != node->end; p++) {
                total_isects++;
                bool p_hit = (*p)->intersect(ray, i);
                hit = hit || p_hit; 
                              
            }
            
            return hit;
        }
        else {
            bool hit_left = intersect(ray, i, node->l); 
            bool hit_right = intersect(ray, i, node->r);
            return hit_left || hit_right;
        }
    
    } else {
        return false;
    }
    
   
        /*
    // prev code
    bool hit = false;
    for (auto p : primitives) {
        total_isects++;
        hit = p->intersect(ray, i) || hit;
    }
    return hit;
    */
     
    
}

} // namespace SceneObjects
} // namespace CGL
