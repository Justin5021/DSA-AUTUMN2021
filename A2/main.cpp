#include "tree.hpp"
#include <chrono>

using namespace std::chrono;

int main(){
    auto start = high_resolution_clock::now();
	BST<int> t;

	t.add_vertex("A", 1);
	t.add_vertex("B", 3);
	t.add_vertex("C", 5);
	t.add_vertex("D", 10);
	t.add_vertex("G", 20);
	t.add_vertex("E", 15);
	t.add_vertex("H", 30);
	t.add_vertex("F", 17);
	t.add_vertex("A",14);

//	t.add_vertex("v0",7);
//    t.add_vertex("v1",26);
//    t.add_vertex("v2",31);
//    t.add_vertex("v3",15);
//    t.add_vertex("v4",16);
//    t.add_vertex("v5",23);
//    t.add_vertex("v6",35);
//    t.add_vertex("v7",34);
//    t.add_vertex("v8",21);
//    t.add_vertex("v9",25);
//    t.add_vertex("v10",11);
//    t.add_vertex("v11",33);
//    t.add_vertex("v12",21);
//    t.add_vertex("v13",16);
//    t.add_vertex("v14",35);
//    t.add_vertex("v15",44);
//    t.add_vertex("v16",30);
//    t.add_vertex("v17",24);

	cout << boolalpha;
	cout << "Nodes: " << t.num_vertices() << endl;
	cout << "Edges: " << t.num_edges() << endl;
    cout << "Height: " << t.height() << endl;

	cout << "Total Weight: " << t.sum_weight() << endl;
    cout << "G Exists?: " << t.contains("G") << endl;

    cout << "Edges: ";
    for (auto const& x : t.get_edges() ) {
        cout << "{" << x.first << "," << x.second << "} ";
    }
    cout << endl;

    cout << "Are D and G adjacent?: " << t.adjacent("G","D") << endl;

    cout << "Leaves: ";
	for ( auto const& x : t.get_leaves() ) {
	    cout << x << " ";
	}
	cout << endl;

    cout << "Nodes: ";
    for ( auto const& x : t.get_vertices() ) {
    cout << x << " ";
	}
	cout << endl;

    cout << "Path: ";
    for ( auto const& x : t.path("A","A") ) {
        cout << x << " ";
    }
    cout << endl;

    cout << "Great Weighted Path: ";
	for(auto const& x : t.path_with_largest_weight()){
		cout << x << " ";
	}
	cout << endl;

//    t.remove_vertex("B");
//    cout << "Nodes: " << t.num_vertices() << endl;
//    cout << "Edges: " << t.num_edges() << endl;
//    cout << "Height: " << t.height() << endl;
//
//    cout << "Total Weight: " << t.sum_weight() << endl;
//    cout << "B Exists?: " << t.contains("B") << endl;

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    cout << "Time taken by function: "
         << duration.count() << "ms" << endl;
}