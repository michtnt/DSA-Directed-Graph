// i#include "directed_graph.hpp"
#include "directed_graph_algorithms.cpp"

int main() {        
    directed_graph<int> g;

	if(g.num_vertices() != 0) return 1;
    if(g.num_edges() != 0) return 1;

    //  VERTICES
	// the vertex object v1 has an id (i.e., A) and a int-typed weight (i.e. 1.25)
	// A = 0; B = 1; C = 2; D = 3; E = 4;
	vertex<int> v1(0, 800); 
    vertex<int> v2(1, 300); 
	vertex<int> v3(2, 400);
	vertex<int> v4(3, 710);
    vertex<int> v5(4, 221); 

    // adding vertex
	g.add_vertex(v1);
	g.add_vertex(v2);
	g.add_vertex(v3);
    g.add_vertex(v4);
	g.add_vertex(v5);

    // EDGE
    // A = 0; B = 1; C = 2; D = 3; E = 4;
	// adding edge 

	g.add_edge(0,1,600);
	g.add_edge(0,2,900);
	g.add_edge(1,4,3000);
	g.add_edge(2,3,4000);
	g.add_edge(3,2,700);
	g.add_edge(3,0,1);
	g.add_edge(3,4,500);

    // topo sort
    // cout << "topo sort" << endl;
    // vector<vertex<int>> topo_sort = topological_sort(g);
    //    for (vertex<int> v : topo_sort) {
	//  	cout << "(" << v.id << ", " << v.weight << ") ";
	// }
	// cout << endl;

	// strong connected
	// cout << "strong connected" << endl;
    // vector<vector<vertex<int>>> strong_connected = strongly_connected_components(g);
    //    for (vector<vertex<int>> a : strong_connected) {
	// 	   for(vertex<int> v : a ){
	//  	    cout << "(" << v.id << ", " << v.weight << ") ";
	// 	 }
	// 	 	cout << endl;
	// }

	// shortest path
	// cout << "shortest path" << endl;
    // vector<vertex<int>> short_path = shortest_path(g, 0, 3);
    //    for (vertex<int> v : short_path) {
	//  	cout << "(" << v.id << ", " << v.weight << ") ";
	// }

	// low cost delivery
	// int result = low_cost_delivery(g,0);
	// cout << result << endl;

    return 0;
}