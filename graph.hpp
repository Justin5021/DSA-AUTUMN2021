/* Developed in CLion
 * Assignment 1 - Weighted Undirected Graph
 * Author: Justin Chin Wei Kit
 */
#ifndef GRAPH_HPP
#define GRAPH_HPP

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
#include <climits>

using namespace std;

template <typename T> // the template allows the weight of an edge to take any numeric data type (denoted by T).
class Graph {

public:

    /* define your data structure to represent a weighted undirected graph */
    map<string,map<string,T>> adj_list;
    /* A map of maps is used to represent the weighted edges in the adjacency list {A -> B -> weight}
     * This makes iterating over the adjacency list very convenient as it can be split into two maps
     * i.e. { A (key of outer map), B (value of outer map) } and { B (key of inner map), weight (value of inner map}
     * Edge weight values are accessed as adj_list[u][v] = val;
    */
    int edges_num;

    /* Test1 */
    Graph(); // the constructor function.
    ~Graph(); // the destructor function.
    size_t num_vertices(); // returns the total number of vertices in the graph.
    size_t num_edges(); // returns the total number of edges in the graph.
    bool vec_find(const string&, vector<string>); // utility function of the vector find algorithm (used often)

    /* Test2 */
    void add_vertex(const string&); // adds a vertex to the graph -- every vertex uses a string as its unique identifier.
    bool contains(const string&); // checks if a vertex is in the graph -- returns true if the graph contains the given vertex; otherwise, returns false.

    /* Test3 */
    vector<string> get_vertices(); // returns a vector of all the vertices in the graph.

    /* Test4 */
    void add_edge(const string&, const string&, const T&); // adds a weighted edge to the graph -- the two strings represent the incident vertices; the third parameter represents the edge's weight.
    bool adjacent(const string&, const string&); // check if there is an edge between the two vertices in the graph -- returns true if the edge exists; otherwise, returns false.

    /* Test5 */
    vector<pair<string,string>> get_edges(); // returns a vector of all the edges in the graph -- each edge is represented by a pair of vertices incident to the edge.

    /* Test6 */
    vector<string> get_neighbours(const string&); // returns a vector of all the vertices, each of which is directly connected with the given vertex by an edge.
    size_t degree(const string&); // returns the degree of a vertex.

    /* Test7 */
    void remove_edge(const string&, const string&); // removes the edge between two vertices, if it exists.

    /* Test8 */
    void remove_vertex(const string&); // delete the given vertex from the graph -- note that, all incident edges of the vertex should be deleted as well.

    /* Test9 */
    vector<string> depth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a depth-first traversal from the given vertex.

    /* Test10 */
    vector<string> breadth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a breadth-first traversal from the given vertex.

    /* Test11 */
    bool find_vector_pair(const string&, vector<pair<string,string>>); // find a specific pair given a specific (target) vertex to look for in a pair vector
    bool contain_cycles(); // check if the graph contains any cycle -- returns true if there exists a path from a vertex to itself; otherwise, return false.

    /* Test12 */
    Graph<T> minimum_spanning_tree(); // returns a spanning tree of the graph -- the returned tree is preferably a minimum spanning tree.

};

/* Test1 */

template <typename T>
Graph<T>::Graph() {
    edges_num = 0; // initialized to 0 to prevent it from becoming a random number
}

template <typename T>
Graph<T>::~Graph() {} // unused destructor

template <typename T>
size_t Graph<T>::num_vertices() {
    return adj_list.size(); // the number of vertex's is equivalent to the adjacency list length
}

template <typename T>
size_t Graph<T>::num_edges() {
    return edges_num; // returns the number of edges found in the list
}

template <typename T>
bool Graph<T>::vec_find(const string& target, vector<string> vec) {
    return find(vec.begin(), vec.end(), target) != vec.end(); // check if the target string can be found in a vector
}

/* Test2 */

template <typename T>
void Graph<T>::add_vertex(const string& u) {
    adj_list[u] = map<string,T>(); // creates a new vertex with an empty inner map key and weight value linked to vertex "u"
}

template <typename T>
bool Graph<T>::contains(const string& u) {
    auto iter = adj_list.find(u);    // the iterator looks throughout the list to find "u"
    if ( iter != adj_list.end() ){   // if the iterator does not point to the end of the map, the value exists in the list
        return true;
    } else
        return false;
}

/* Test3 */

template <typename T>
vector<string> Graph<T>::get_vertices() {
    vector<string> vertices;            // vector to store all the vertexes
    for ( auto const& x : adj_list ) {  // C++11 iterate/loop over the whole map
        vertices.push_back(x.first);    // x represents the outer map, x.first is the key of the outer map (vertex)
    }
    return vertices;
}

/* Test4 */

template <typename T>
void Graph<T>::add_edge(const string& u, const string& v, const T& weight) {
    if ( contains(u) && contains(v) ) { // checks if list contains vertices u and v first
        adj_list[u][v] = weight;        // assign the weight value to the corresponding [outer map key][inner map key]
        adj_list[v][u] = weight;        // undirected graph, so do it both ways
        ++edges_num;                    // increment the number of edges
    }
}

template <typename T>
bool Graph<T>::adjacent(const string& u, const string& v) {
    if ( contains(u) && adj_list[u].find(v) != adj_list[u].end() )
        // checks if "u" exists and if a vertex of key "v" can be found in the adjacency list of "u" (adjacent vertex to u)
        return true;
    else
        return false;
}

/* Test5 */

template <typename T>
vector<pair<string,string>> Graph<T>::get_edges() {
    vector<string> vertices;                                // vector to exclude duplicate edges (undirected graph, so stored edges are doubled)
    vector<pair<string,string>> edges_pairs;                // pair vector to store unique edges
    for ( auto const& outer : adj_list ) {                  // loop over the list to get the outer map key and value
        for ( auto const& inner : adj_list[outer.first] ) { // find the children of the first key value
            if ( !vec_find(inner.first, vertices) ) {       // if the key value has not been added previously
                edges_pairs.push_back(make_pair( outer.first , inner.first)); // store the {outer map vertex, adjacent vertex (inner map key)} as a pair
            }
        }
        vertices.push_back(outer.first);
    }
    return edges_pairs;
}

/* Test6 */

template <typename T>
vector<string> Graph<T>::get_neighbours(const string& u) {
    vector<string> vertices;                     // to store all the adjacent vertices from the target vertex u
    if ( contains(u) ) {                         // checks if the vertex exists
        for ( auto const& x  : adj_list[u] ) {   // loops throughout the vertices (outer map) to get the inner map keys (x.first)
            vertices.push_back(x.first);
        }
    }
    return vertices;
}

template <typename T>
size_t Graph<T>::degree(const string& u) {  // is the number of incident edges given a target vertex "u"
    return adj_list[u].size();              // which is also the number of vertices found adjacent to it
}

/* Test7 */

template <typename T>
void Graph<T>::remove_edge(const string& u, const string& v) {
    if ( contains(u) && contains(v) ) { // checks if list contains vertices u and v first
        adj_list[u].erase(v);           // the outer map vertex must erase the inner map key and value
        adj_list[v].erase(u);           // undirected graph, so the reverse must be erased as well
        --edges_num;                    // decrement the number of edges
    }
}

/* Test8
 * A has an edge to B, C and E
 * Vertices B, C and E must contain A then
 * Delete "key "A" and weight" from B, C and E so B->{A,weight}, C->{A,weight} and E->{A,weight} are gone
 * Now delete A completely, so all inner map contents are deleted
 */

template <typename T>
void Graph<T>::remove_vertex(const string& u) {
    if ( contains(u) ) {                        // if the vertex exists in the list
        for ( auto const& x : adj_list[u] ) {   // iterate over every adjacent vertex
            adj_list[x.first].erase(u);         // erase targeted "u" key and its corresponding value for every neighbour of u
            --edges_num;                        // reduce the edges count for each map erased
        }
        adj_list.erase(u);                      // completely erase u, and that will remove all the neighbours linked to u
    }
}

/* Test9
 * Iterative implementation of DFS
 * The created "stack" was done as a vector for convenience
 * Since most of the implemented functions used similar find algorithms from vector's already
 * Vectors can also be iterated whereas a stack does not have random access because it is LI-FO
 * The stack and later queue was done as a vector to prevent duplicate vertices from entering the unprocessed list
 */

template <typename T>
vector<string> Graph<T>::depth_first_traversal(const string& u) {
    vector<string> visited;                     // vector to store the vertices of the visited traversal path
    vector<string> s;                           // called it stack (s) because it is dfs for convenience
    s.push_back(u);                             // u (vertex) is the starting point point of the search to be processed
    while ( !s.empty() ) {                      // while there are still values to be processed in the stack
        string curr_vert = s.back();            // top most vertex (back) is added to be processed
        s.pop_back();                           // and remove it from the stack since it will be processed
        if ( !vec_find(curr_vert, visited) ) {  // checks if target has not been visited yet
            visited.push_back(curr_vert);       // adds the vertex to the visited list if it has not been visited
        }                                       // iterate over every vertex adjacent to the current vertex
        for ( auto const& x : adj_list[curr_vert] ) {
            if ( !vec_find(x.first, visited) && !vec_find(x.first, s) ) {
                s.push_back(x.first);           // if the adjacent vertex has not been visited before and is not in the stack, push it to be processed.
            }
        }
    }
    return visited;
}

/* Test10
 * Iterative implementation of BFS
 * The queue was also made using a vector rather than std::queue for convenience of searching through it
 * In return, the dfs and bfs code are nearly identical and the biggest change is the insertion into the unprocessed queue of vertices
 * Because the values are taken and removed from the back of the vector, this means, the "back" of our vector can be treated as the front
 * Vice-versa, the front of the vector can be treated as the back of the queue (if it was a real queue)
 */

template <typename T>
vector<string> Graph<T>::breadth_first_traversal(const string& u) {
    vector<string> visited;                     // store traversal path
    vector<string> q;                           // named queue (q) because it is bfs for convenience
    q.push_back(u);                             // u (vertex) is the starting point point of the search to be processed
    while ( !q.empty() ) {                      // while there are still values to be processed in the queue
        string curr_vert = q.back();            // vertex is added to be processed
        q.pop_back();                           // and is removed from the queue
        if ( !vec_find(curr_vert, visited) ) {  // checks if target has not been visited yet
            visited.push_back(curr_vert);       // adds the vertex to the visited list if it has not been visited
        }                                       // iterate over every vertex adjacent to the current vertex
        for ( auto const& x : adj_list[curr_vert] ) {
            if ( !vec_find(x.first, visited) && !vec_find(x.first, q) ) {
                q.insert(q.begin(),x.first);    // if the adjacent vertex has not been visited before and is not in the queue, push it to be processed.
            }                                   // because it is a vector, not a queue, the vertex is added in the front (reverse) of the vector to emulate the effect
        }
    }
    return visited;
}

/* Test11 */

template <typename T>
bool Graph<T>::find_vector_pair(const string& target, vector<pair<string,string>> vector_pair) {
    for ( auto const& x : vector_pair ) {   // loop over the pair vector contents
        if ( x.first == target ) {          // check if a target string matches the pair.first of the element in the vector
            return true;                    // return true if the matching pair can be found
        }
    }
    return false;
}

/* The cycles code utilizes the code from dfs, hence why a "stack" is created
 * The vector, will store the information of the parent and value in its map as a pair
 */

template <typename T>
bool Graph<T>::contain_cycles() {
    vector<string> visited;                            // keeps track of what has been visited
    vector<pair<string,string>> s;                     // stores a pair where the pair.first value is the child of the original vertex searched
    s.push_back(make_pair(get_vertices().front(),"")); // starts the dfs a random vertex taken from the front of the list
    while ( !s.empty() ) {
        string curr_vert = s.back().first;
        string parent_vert = s.back().second;          // this is called parent_vert, because the parent of the original vertex will be stored as pair.second
        s.pop_back();
        visited.push_back(curr_vert);                  // curr_vert is now seen and is added to traversed vertices list
        for ( auto const& x : adj_list[curr_vert] ) {
            if ( !vec_find(x.first, visited) && !find_vector_pair(x.first, s) ) {
                s.push_back(make_pair( x.first , curr_vert));
                // if the vertex (child of curr_vert) has not been seen
                // and is not found to be a string in any of the pairs of stack located in pair.first
                // then add it to the stack as a pair (flipped) where the child is pair.first and curr_vert is pair.second
                // the roles are flipped so that in the next iteration, there are extensive paths that can be considered using the new pair.first
            } else if ( x.first != parent_vert )       // if the child has been "seen" or was found as a "parent" to another vertex
                return true;                           // cycle has been found (back-edge)
        }
    }
    return false;
//    if ( num_edges() >= (num_vertices()-1) )
//        return true;
//    else
//        return false;
}

/* Test12
 * Implemented based on Prim's Algorithm from Week 5 Slides
 */

template <typename T>
Graph<T> Graph<T>::minimum_spanning_tree() {
    Graph<int> mst;
    vector<string> visited;                              // vector to store previously visited vertices to prevent it from interfering with new vertices
    mst.add_vertex(get_vertices().front());              // adds the first key in the adj_list to mst graph
    visited.push_back(get_vertices().front());           // the key is the starting vertex to check its children's minimum weights
    while ( mst.num_vertices() < num_vertices() ) {      // Prim's Algorithm condition, mst vertices cannot exceed the original graphs number
        int min_val = INT_MAX;                           // the minimum value is currently the biggest integer value possible
        pair<string,string> path;                        // to store the cheapest path based on vertices adjacency
        for ( auto const& outer : visited ) {
            for ( auto const& inner : adj_list[outer]) {
                if ( !vec_find(inner.first, visited) && (inner.second < min_val) ) {
                    min_val = inner.second;
                    path = make_pair(outer,inner.first);
                    // if the inner map (child) key has not been visited before, and is smaller than the largest integer
                    // assign that value, and then use the pair to store the {outer map key,inner map key}
                    // that pair may form the nex lowest cost edge in the mst graph, but the loop is done once again for the next child to check if
                    // that child's edge weight value is lower than the current one in min_val
                }
            }
        }
        visited.push_back(path.second);                 // take the resulting pair's adjacent vertex and push it into visited to find its children next
        mst.add_vertex(path.second);                    // new vertex in mst graph using the resultant pair
        mst.add_edge(path.first,path.second,adj_list[path.first][path.second]); // add a new edge in mst using the pair
    }                                                   // the weight value is assigned from adj_list[u][v] from the original graph
    return mst;
}

#endif