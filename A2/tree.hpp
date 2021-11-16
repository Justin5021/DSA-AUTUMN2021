/* Developed in CLion
 * Assignment 2 - Binary Search Tree
 * Author: Justin Chin Wei Kit
 */
#ifndef DSA_A2_TREE_HPP
#define DSA_A2_TREE_HPP

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <queue>
#include <stack>
#include <limits>
#include <utility>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <cmath>

using namespace std;

template <typename T>
class Node {
public:
    string vertex;                  // identifier of node must be unique
    T weight;                       // weight of node
    Node *left;                     // left child pointer
    Node *right;                    // right child pointer
    Node(const string&, const T&);  // node constructor that takes a string and T val for name and weight
    ~Node();                        // destructor to delete left and right children
};

template <typename T> // the template allows the weight of vertex to take any numeric data type (denoted by T).
class BST {

public:

    /* define your data structure to represent a binary search tree (bst) */
    Node<T> *root = new Node<T>("u",0); // random values to satisfy node constructor, BST constructor assigns this to null
                                        // root node will be the first node added to the tree
    map<string,T> vertices;
    /* Map to contain identifiers and the weights of each corresponding vertex
     * Provides quicker access to the weights of a specific node since vertices[node] = weight
     * Faster than creating a get_weight helper function to do it iterative/recursively (slower)
     * Makes certain functions like get_vertices or contains (etc.) much simpler to write and faster
     */
    T total_weight; // total weightage of the tree, also reduces needing an extra traversal

    /* Test 1 */
    BST();                  // the constructor function.
    ~BST();                 // the destructor function.
    size_t num_vertices();  // returns the total number of vertices in the bst.
    size_t num_edges();     // returns the total number of edges in the bst.
    T sum_weight();         // return the total weight of all the vertices in the bst.

    /* Test 2 */
    void add_vertex(const string&, const T&); // adds a vertex, which has a weight, to the tree -- every vertex uses a string as its unique identifier.
    bool contains(const string&);             // checks if a vertex is in the bst -- returns true if the bst contains the given vertex; otherwise, returns false.

    /* Test 3 */
    vector<string> get_vertices(); // returns a vector of all the vertices in the bst.
    vector<string> get_leaves();   // returns a vector of all the leaves in the bst.
                                   // Leaves are the vertices that do not have any children in the bst.

    /* Test 4 */
    bool adjacent(const string&, const string&); // check if there is an edge between the two vertices in the bst -- returns true if the edge exists; otherwise, returns false.

    /* Test 5 */
    vector<pair<string,string>> get_edges(); // returns a vector of all the edges in the bst -- each edge is represented by a pair of vertices incident to the edge.

    /* Test 6 */
    vector<string> get_neighbours(const string&); // returns a vector of all the vertices, each of which is directly connected with the given vertex via an edge.
    size_t degree(const string&);                 // returns the degree of a vertex.

    /* Test 7 */
    void preorder_helper(const Node<T>*, vector<string>&); // preorder helper to do the traversal recursively to make it easier to give it a node input
    vector<string> preorder_traversal();                   // returns a vector of all the vertices in the visiting order of a preorder traversal over the bst.

    /* Test 8 */
    void inorder_helper(const Node<T>*, vector<string>&); // similar helper but for the inorder traversal: left->current->right node
    vector<string> inorder_traversal();                   // returns a vector of all the vertices in the visiting order of an inorder traversal over the bst.

    /* Test 9 */
    void postorder_helper(const Node<T>*, vector<string>&); // similar helper but for the postorder traversal: left->right->current node
    vector<string> postorder_traversal();                   // returns a vector of all the vertices in the visiting order of a postorder traversal over the bst.

    /* Test 10 */
    vector<string> breadth_first_traversal(); // returns a vector of all the vertices in the visiting order of a breadth first traversal over the bst.

    /* Test 11 */
    bool get_path(const Node<T>*, const string&, vector<string>&); // helper to check if there is an available path (bool) between the root and target node
    vector<string> path(const string&, const string&);             // returns a vector of all the vertices in the path from the first vertex to the second vertex.
                                                                   // A path should include the source and destination vertices at the beginning and the end, respectively.

    /* Test 12 */
    vector<string> path_with_largest_weight(); // returns a path that has the largest weight in the bst.
                                               // The weight of a path is the sum of the weights of all the vertices (including the source and destination vertices) in the path.

    /* Test 13 */
    size_t height_helper(const Node<T>*); // recursively increment a counter to find the number of layers in the tree
    size_t height();                      // returns the height of bst. Height of a tree is the number of edges that form the longest path from root to any leaf.

    /* Test 14 */
    Node<T>* remove_helper(Node<T>*, const string&); // helper function to find a specific matching node (identifier) and then remove it
    void remove_vertex(const string&);               // delete the given vertex from bst -- note that, all incident edges of the vertex should be deleted as well.
};

/* Test 1 */

template <typename T>
Node<T>::Node(const string& identifier, const T& w) {
    vertex = identifier;
    weight = w;
    left = NULL;  // when using the constructor, the left and right children
    right = NULL; // should be automatically set to null as it has no information yet
}

template <typename T>
Node<T>::~Node() {
    delete left; // when node is deleted, deallocate the memory
    delete right; // of the left and right children
}

template <typename T>
BST<T>::BST() {
    root = NULL; // tree should start empty
    total_weight = 0; // initialized to 0 to prevent it from becoming a random number
}

template <typename T>
BST<T>::~BST() {
    delete root; // when the tree is no longer in use, deallocate the memory of the root
}

template <typename T>
size_t BST<T>::num_vertices() {
    return vertices.size(); // the size of the map is equivalent to the number of vertices in the tree
}

template <typename T>
size_t BST<T>::num_edges() {
    if ( root == NULL )             // check if the tree is empty first
        return 0;                   // prevents returning a negative value as size_t only accepts positive integers
    else
        return num_vertices() - 1;  // this will always be true because of our data structure only allowing 2 children per vertex
}

template <typename T>
T BST<T>::sum_weight() {
    return total_weight; // returns the total weight variable
}

/* Test 2 */

template <typename T>
void BST<T>::add_vertex(const string& u, const T& w) {
    if ( !contains(u) ) {                           // check for uniqueness in the tree, ignore adding if identifier is found and exists
        Node<T> *temp_node = new Node<T>(u,w);      // assign a temporary node to take the newly created values using node constructor
        if ( root == NULL )                         // if the tree was empty still
            root = temp_node;                       // root takes the created pointers value
        else {
            Node<T> *curr_node = root;              // let the current node be the root to first check for the children which are currently pointing to nothing
            Node<T> *parent_node = NULL;            // to make it easier to identify the parent/child
            // navigate through the sub-tree to find the node
            // where we want to add the new node as the left or right child
            while ( curr_node != NULL ) {           // condition to find the node whose children currently points to nothing (empty) "the next open space"
                parent_node = curr_node;            // stores the parent node which will be used to add its children later
                if ( w < curr_node->weight )        // condition to navigate through the left sub-tree
                    curr_node = curr_node->left;    // assign it the left child
                else                                // otherwise the node should be added in the right sub-tree
                    curr_node = curr_node->right;   // assign it the right child
            }                                       // repeats until it navigates to an empty spot in the sub-tree which matches the conditions of BST
            if ( w < parent_node->weight )          // BST condition, weight < the nodes weight it will be left child
                parent_node->left = temp_node;      // left child
            else
                parent_node->right = temp_node;     // if weight > node weight, then it is the right child
        }
        vertices[u] = w;        // store into the map as {u,w}
        total_weight += w;      // increment the total weight of the tree
    }
}

template <typename T>
bool BST<T>::contains(const string& u) {
    return vertices.find(u) != vertices.end(); // checks through vertices map to see if it is stored O(log(n)) time
}

/* Test 3 */

template <typename T>
vector<string> BST<T>::get_vertices() {
    vector<string> verts;             // vector to store all the nodes from the map
    for ( auto const& x : vertices )  // iterate over the map
        verts.push_back(x.first);     // push the key values of the map into verts
    return verts;                     // return it
}

/* Starting from an initial node (root) keep adding subsequent left/right children
 * And only add it to the final "leaves" vector if that node's left and right children are empty
 * After that or otherwise, add the subsequent children to process as long as they are not null
 */

template <typename T>
vector<string> BST<T>::get_leaves() {
    if ( root == NULL )                                              // to prevent access violation error
        return {};                                                   // return empty vector
    vector<string> leaves;                                           // store the leaves
    vector<Node<T>*> q;                                              // vector to process all the nodes in the tree
    q.push_back(root);                                               // starting node
    while ( !q.empty() ) {                                           // while there are values to be processed
        Node<T> *curr_node = q.back();                               // back-most node used to check its children status
        q.pop_back();                                                // pop it out of the queue since it will be dealt with
        if ( curr_node->left == NULL && curr_node->right == NULL ) { // if both left and right child are empty
            leaves.push_back(curr_node->vertex);                     // it is a leaf so add it to the leaves vector (no children)
        }
        if ( curr_node->left != NULL )                               // if left child is not null add it to be processed
            q.push_back(curr_node->left);
        if ( curr_node->right != NULL )                              // if right child is not null add it to be processed
            q.push_back(curr_node->right);
    }
    return leaves;
}

/* Test 4
 * Utilizes the get_edges() function because any pair that was added into get_edges()
 * Must be a legitimate edge pair in the tree
 * Parameters u and v are checked to see if it is the first and second value of the pair
 * Vice versa option included because it could be the child-parent (v,u) rather parent-child (u,v)
 */

template <typename T>
bool BST<T>::adjacent(const string& u, const string& v) {
    if ( !contains(u) || !contains(v) )   // check to see if u and v exists
        return false;                     // if one doesn't, return false
    for ( auto const& p : get_edges() ) { // utilizes get_edges, a pair in get_edges() means they have to be adjacent already
        if ( ( p.first == u && p.second == v ) || ( p.first == v && p.second == u ) )
            return true;                  // v could be the parent instead of u hence "or" and return true instantly once found
    }
    return false;
}

/* Test 5
 * It will keep adding the children of the nodes
 * Until it reaches a point where the node being checked
 * Does not have a left child nor right child (leaf node)
 * As long as the child exists (left OR right) it will make a pair of (parent,child)
 */

template <typename T>
vector<pair<string,string>> BST<T>::get_edges() {
    if ( root == NULL )                         // to prevent access violation error
        return {};
    vector<Node<T>*> q;                         // queue to process the children of the nodes
    vector<pair<string,string>> edges_pair;     // the vector pair to return
    q.push_back(root);
    while ( !q.empty() ) {
        Node<T> *curr_node = q.back();          // Top/back-most node in the queue
        q.pop_back();                           // pop it because it will be processed
        if ( curr_node->left != NULL) {         // if left child is not empty
            edges_pair.push_back(make_pair(curr_node->vertex,curr_node->left->vertex)); // make a pair with its parent
            q.push_back(curr_node->left);       // add left child to make following pairs
        }
        if ( curr_node->right != NULL ) {       // if right child is not empty
            edges_pair.push_back(make_pair(curr_node->vertex,curr_node->right->vertex)); // make a pair with its parent
            q.push_back(curr_node->right);      // add right child to make following pairs
        } // if both left and right child were null (this would be the condition to be a leaf (get_leaves()))
    }
    return edges_pair;
}

/* Test 6
 * Utilizes get_edges() similar to adjacency, all neighbours of a node must be adjacent
 * Find all instances where the node is a parent (pair.first) or child (pair.second) and it is a neighbour
 */

template <typename T>
vector<string> BST<T>::get_neighbours(const string& u) {
    vector<string> neighbours;
    if ( contains(u) ) {                        // check that the node exists first
        for ( auto const& p : get_edges() ) {
            if ( p.first == u ) {               // if u is a parent in a pair from get_edges()
                neighbours.push_back(p.second); // add the child in the pair
            }
            if ( p.second == u ) {              // if u is a child in a pair from get_edges()
                neighbours.push_back(p.first);  // add the parent in the pair
            }
        }
    }
    return neighbours;
}

template <typename T>
size_t BST<T>::degree(const string& u) {
    return get_neighbours(u).size(); // degree is the number of nodes connected to target node parent/child of the node
}

/* Test 7
 * All the traversal helpers are identical, just the position of when the node is pushed into the vector is changed
 * push->left tree->right tree
 */

template <typename T>
void BST<T>::preorder_helper(const Node<T>* node, vector<string>& vec) {
    if ( node == NULL )                  // the condition to stop the traversal
        return;
    vec.push_back(node->vertex);         // current node should be pushed into vec first before traversing its children
    preorder_helper(node->left,vec);  // left sub-tree is first traversed
    preorder_helper(node->right,vec); // then right sub-tree is traversed
}

template <typename T>
vector<string> BST<T>::preorder_traversal() {
    vector<string> path;                 // vector to be returned where the helper adds the path
    preorder_helper(root,path); // call the helper
    return path;
}

/* Test 8
 * left tree->push->right tree
 */

template <typename T>
void BST<T>::inorder_helper(const Node<T>* node, vector<string>& vec) {
    if ( node == NULL )                  // condition to stop the traversal
        return;
    inorder_helper(node->left,vec);   // left sub-tree is traversed first before
    vec.push_back(node->vertex);         // the current node (with no more left children) is added
    inorder_helper(node->right,vec);  // then traverse right sub-tree
}

template <typename T>
vector<string> BST<T>::inorder_traversal() {
    vector<string> path;                 // the returned path
    inorder_helper(root,path);  // calls the inorder_helper
    return path;
}

/* Test 9
 * left tree->right tree->push
 */

template <typename T>
void BST<T>::postorder_helper(const Node<T>* node, vector<string>& vec) {
    if ( node == NULL )                    // condition to stop the traversal
        return;
    postorder_helper(node->left,vec);   // left sub-tree is traversed
    postorder_helper(node->right,vec);  // then the right sub-tree
    vec.push_back(node->vertex);           // only then is the current node added
}

template <typename T>
vector<string> BST<T>::postorder_traversal() {
    vector<string> path;
    postorder_helper(root,path);
    return path;
}

/* Test 10
 * The process of BFS is very similar to the code in get_edges and get_leaves
 * Instead of checking for both children being null (leaf) or making a pair with its parent
 * It just adds nodes it visits in the left->right order as it progresses down the layers of the tree
 */

template <typename T>
vector<string> BST<T>::breadth_first_traversal() {
    vector<string> visited;                         // store traversal path
    queue<Node<T>*> q;                              // queue of nodes to process the nodes
    q.push(root);                                   // starting node
    while ( !q.empty() ) {
        Node<T> *curr_node = q.front();             // temporarily assign it the front value from the queue (FIFO)
        q.pop();                                    // remove it as it will be processed
        visited.push_back(curr_node->vertex);       // add the current node to visited
        if ( curr_node->left != NULL )              // if left child exists, add it first because BFS goes from left->right between layers
            q.push(curr_node->left);                // push it into the queue to process
        if ( curr_node->right != NULL )             // continue the same process for right child
            q.push(curr_node->right);
    }
    return visited;
}

/* Test 11
 * The get_path function essentially checks for the viability of a path from (most likely) the root to the node
 * It will keep forming its path using recursion until it stops (node == NULL) or it found the matching identifier
 * If all those conditions fail, it will pop out the added vertex to make sure it does not form an incorrect path
 */

template <typename T>
bool BST<T>::get_path(const Node<T>* node, const string& u, vector<string>& vec) {
    if ( node == NULL )          // condition to stop the recursion
        return false;            // and if the root node was NULL (empty tree), there would be no valid path from root to u
    vec.push_back(node->vertex); // add the node into the vector (this will form the actual path)
    if ( node->vertex == u )     // if the current node's identifier matches u
        return true;             // the path is completed and is a valid path
    if ( get_path(node->left,u,vec) || get_path(node->right,u,vec) ) // otherwise recurse until node u is found in either left/right sub-trees
        return true;             // if either of those children's identifier matches with u, it is a valid path
    vec.pop_back();              // if all fails, remove the node because it is not a valid path from root to u
    return false;
}

/* After the paths of root to u and root to v are found
 * Add the u_path order in reverse (aka from the front of v_path) because we want to flip u_path's order
 * Before adding v_path into the final vector, duplicate keys must be filtered out
 * The intersection check is needed because we only want the node where both paths can meet "once"
 * Hence a count is included to ensure that duplicates are excluded, and the first duplicate
 * Is added only if intersection is still true (aka has not been found)
 */

template <typename T>
vector<string> BST<T>::path(const string& u, const string& v) {
    if ( !contains(u) || !contains(v) )                      // check if u and v both exist in the tree
        return {};                                           // if one doesn't, return an empty vector
    vector<string> merged, u_path, v_path;                   // 3 vectors to store their respective paths
    get_path(root,u,u_path);                        // get the path from root to u
    get_path(root,v,v_path);                        // get the path from root to v
    bool intersection = true;                               // to make sure the intersection (the root) will be included in the final vector
    for ( auto const& x : u_path )                          // for each element in u_path, add it to v_path from the front
        v_path.insert(v_path.begin(),x);                    // insert from the front of v_path to maintain the proper traversal order
    for ( auto const& vert : v_path) {
        if (count(v_path.begin(), v_path.end(), vert) == 1) // this is the check to ignore the duplicates
            merged.push_back(vert);                         // add the nodes into the final vector "merged"
        // when a duplicate is found (count != 1), we check if the intersection was previously found
        // If it hasn't been found before, add the current node to the final vector
        else if (intersection) {                            // check the intersection status
            merged.push_back(vert);                         // push that vertex into the vector to include the intersection
            intersection = !intersection;                   // set the intersection to false because intersection was found and added already
        }
    }
    return merged;
}

/* Test 12
 * The heaviest path, is most likely to occur from one leaf to another leaf
 * So create a path using x (all the nodes in the tree) and y (all the leaves in the tree)
 * Find that specific paths total weight and run a check with the current "heaviest" path weight
 * After it checks every path, the largest path is returned
 */

template <typename T>
vector<string> BST<T>::path_with_largest_weight() {
    vector<string> largest_path;
    int greatest = 0;                           // value to compare to find the heaviest path
    for ( auto const& x : get_vertices() ) {    // all nodes in the tree
        for (auto const& y : get_leaves() ) {   // all leaves in the tree
            int curr_total = 0;                 // generated total weight for each path
            for ( auto const& p : path(x,y) ) { // for each node in the path "p"
                curr_total += vertices[p];      // add all the weights to the running total
            }
            if ( curr_total > greatest ) {      // condition to check
                greatest = curr_total;          // the new greatest value is the current total if it is true
                largest_path = path(x,y);       // the new largest path would be the one that passes the condition
            }                                   // once all paths have been checked, the final largest_path will be returned
        }
    }
    return largest_path;
}

/* Test 13
 * The code finds the number of layers by comparing the left and right sub-trees
 * This will keep comparing each sub-tree at different layers of the tree
 * And constantly does max(left_tree_layer, right_tree_layer) + 1
 * Until it reaches the leaves because node->left and node->right would be empty
 */

template <typename T>
size_t BST<T>::height_helper(const Node<T>* node) {
    if ( node == NULL ) { // condition to stop the traversal or when there is no new layer
        return 0;         // return 0 instead of -1 because if root == NULL, size_t will give an error as it only accepts >= 0
    }                     // to compensate a (-1) is added in the main function
    return max(height_helper(node->left),height_helper(node->right)) + 1; // increment the max value to make sure it goes up by 1 layer (counting-wise)
}

template <typename T>
size_t BST<T>::height() {
    return (root == NULL) ? 0 : height_helper(root) - 1; // again size_t only allows >= 0, so if the tree is empty, return 0
}  // empty tree, return 0, otherwise height = number of layers - 1

/* Test 14
 * The code firstly, finds the node we are looking for by referring to its weight (BST node adding condition)
 * Once found, it will check its children status, 0 (leaf), 1 or 2 children
 * Depending on its case, do the appropriate changes to the tree
 * For 2 children, it must also find the left-most node in the right sub-tree of the node being deleted
 * And use those values to replace the deleted node
 */

template <typename T>
Node<T>* BST<T>::remove_helper(Node<T>* node, const string& u) {
    if ( node == NULL )                                                 // if the node is null, nothing to delete
        return node;                                                    // return the empty node
    else if ( vertices[u] < node->weight )                              // search for the node based on its weight
        node->left = remove_helper(node->left,u);                       // if found in that area, use the helper on that left-child
    else if ( vertices[u] > node->weight )
        node->right = remove_helper(node->right,u);                     // otherwise use helper on right-child
    else { // when the node has been found because the weight matches vertices[u] == w
        if ( node->left == NULL && node->right == NULL ) {              // it is a leaf, deallocate its memory and set it to null
            delete node;
            node = NULL;
        } else if ( node->left == NULL && node->right != NULL ) {       // right child exists
            node = node->right;                                         // assign the node the right child's values to replace
            delete node->right;                                         // deallocate its memory and assign it NULL
            node->right = NULL;
        } else if ( node->right == NULL && node->left != NULL ) {       // left child exists
            node = node->left;                                          // otherwise use the left child's values
            delete node->left;                                          // deallocate the memory and assign it NULL
            node->left = NULL;
        } else { // 2 children
            Node<T> *temp_node = node->right;                           // access the right sub-tree of the node to be deleted
            while ( temp_node->left != NULL ) {                         // find the left-most node until its left child is null
                temp_node = temp_node->left;                            // replace its value
            }                                                           // once it is finished finding the left-most node
            node->weight = temp_node->weight;                           // replace the weight of node to-be-deleted
            node->vertex = temp_node->vertex;                           // and identifier with the left-most node
            node->right = remove_helper(node->right,temp_node->vertex); // then delete the node that was used
        }
    }
    return node;
}

template <typename T>
void BST<T>::remove_vertex(const string& u) {
    if ( contains(u) ) {                     // check if the node exists first
        total_weight -= vertices[u];         // reduce the total weight of the tree
        root = remove_helper(root,u);  // delete the target node u using the helper function
        vertices.erase(u);                   // delete the corresponding key,value pair from the map
    }
}

#endif