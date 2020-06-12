#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <array>
#include <list>
#include <forward_list>
#include <deque>
#include <map>
#include <cstddef>
#include <string>
#include <utility>
#include <algorithm>
#include <climits>
#include <optional>
#include <exception>
#include <stdexcept>

#include "directed_graph.hpp"

using namespace std;

/*
 * Computes the shortest distance from u to v in graph g.
 * The shortest path corresponds to a sequence of vertices starting from u and ends at v,
 * which has the smallest total weight of edges among all possible paths from u to v.
 */

// topological-sort
stack<vertex<int>> topo_stack;

// strongly-connected-components
stack<vertex<int>> scc_stack;
vector<vertex<int>> each_scc;
vector<vector<vertex<int>>> scc_result;

// shortest-paths
vector<vertex<int>> each_path;
vector<vector<vertex<int>>> all_paths;

template <typename T>
vector<vertex<T>> shortest_path(directed_graph<T> g, int u_id, int v_id) {
    int matrixSize = g.get_vertices().size();
    vector<bool> visited(matrixSize, false); // initialise all visited to false  
	map<int, map<int, int> > matrix; // matrix to determine which vertex are connected by edges`

	vector<vertex<T>> result;

    int *path = new int[matrixSize]; // to store paths collection
    int index = 0; // Initialize path[] as empty 

	// initialise matrix
    for(edge<T> e: g.get_edges()){
		matrix[e.first][e.second] = e.weight;
	}

	// variables needed to find minimum path
    int max = INT_MAX; 
	int total = 0; // current total path weight
	int position = 0; // to know which path are we on from u_id to v_id possible paths collections
    
	// get all possible paths
    getAllPaths(u_id, v_id, visited, path, index, g); 

    for (vector<vertex<int>> p : all_paths) {
		 for(int i = 0; i < p.size()-1; i++){
			 total += matrix[p[i].id][p[i+1].id];
		 }
		 // to help get the minimum
		 if(total < max){
			 max = total;
			 result.clear(); // clear it so it did not add up
			 result = all_paths[position]; // get the path collection
		 }  

		 // reinilization
		 total = 0;
		 position++;
	}
    
    return result;
	
}

template <typename T>  
void getAllPaths(int u, int d, vector<bool> &visited, int path[], int &index, directed_graph<T> g){ 
	 vector<vertex<T>> vertice_list = g.get_vertices();
     map<int, map<int, int> > matrix; // matrix to determine which vertex are connected by edges`
	
	 // Mark the current node and store it in path[] 
	 visited[u] = true; 
	 path[index] = u; 
	 index++; 

	 // initialise matrix for this g
     for(edge<T> e: g.get_edges()){
		matrix[e.first][e.second] = 1;
	}

	// if source == destination then stop, push the path to the current path collection
    if (u == d) { 
        for (int i = 0; i<index; i++){
            each_path.push_back(vertex<T>(path[i], vertice_list[path[i]].weight));
        }
            all_paths.push_back(each_path); // push to the collections
            each_path.clear(); // clear each path after adding so it doesn't add up      
    } 
    else { 
	    // iterate to vertices adjacent to v
        for (vertex<T> adj : vertice_list) 
            if (matrix[u][adj.id] == 1 && !visited[adj.id]) 
                getAllPaths(adj.id, d, visited, path, index, g); 
    } 

    // Remove current vertex from path[] and mark it as unvisited 
    index--; 
    visited[u] = false;
}

/*
 * Computes the strongly connected components of the graph.
 * A strongly connected component is a subset of the vertices
 * such that for every pair u, v of vertices in the subset,
 * v is reachable from u and u is reachable from v.
 */

// Kosaraju's algorithm
template <typename T>
vector<vector<vertex<T>>> strongly_connected_components(directed_graph<T> g) {
	vector<vertex<T>> vertice_list = g.get_vertices();
	map<int, map<int, int> > matrix; // matrix to determine which vertex are connected by edges`

	vector<bool> visited(vertice_list.size(), false); // initialise all g visited to false  
	vector<bool> transpose_visited(vertice_list.size(), false); // initialise all g transposed visited to false

	// fill vertices in stack according to when they are ending
	  for(vertex<T> v : vertice_list){
		if(!visited[v.id]){
			scc_util(v, visited, g);
		}
     }

	 // unload the stack and push it to scc_result
	 while(!scc_stack.empty()){
		 vertex<T> v = scc_stack.top();
		 scc_stack.pop();

		 if(!transpose_visited[v.id]){
			 scc_dfs(v, transpose_visited, g); // iterate on the reversed graph
			 scc_result.push_back(each_scc); // push each scc to final scc result
			 each_scc.clear(); // clear it so it doesn't add up all the other scc
		 }
	 }

	return scc_result;
}

template <typename T>
void scc_util(vertex<T> v, vector<bool> &visited, directed_graph<T> g){
	map<int, map<int, int> > matrix; // matrix to determine which vertex are connected by edges`
    vector<vertex<T>> vertice_list = g.get_vertices();

	visited[v.id] = true;

	// initialise matrix for this graph
	for(edge<T> e: g.get_edges()){
		matrix[e.first][e.second] = 1;
	}

	// iterate to vertices adjacent to v
	for(vertex<T> adj : vertice_list){
		if(matrix[v.id][adj.id] == 1 && !visited[adj.id]){
			scc_util(vertex<T>(adj.id, vertice_list[adj.id].weight), visited, g);
		}
	}

	scc_stack.push(v);
}

template <typename T>
void scc_dfs(vertex<T> v, vector<bool> & transpose_visited, directed_graph<T> g){
	map<int, map<int, int> > matrix; // matrix to determine which vertex are connected by edges`
    vector<vertex<T>> vertice_list = g.get_vertices();

	transpose_visited[v.id] = true;
	each_scc.push_back(v);

	// initialise matrix for this graph, make it reversed
	for(edge<T> e : g.get_edges()){
		matrix[e.second][e.first] = 1; // reversed
		matrix[e.first][e.second] = 0; // normal
	 }

	// iterate to vertices adjacent to v
	for(vertex<T> adj : vertice_list){
		if(matrix[v.id][adj.id] == 1 && !transpose_visited[adj.id]){
			scc_dfs(vertex<T>(adj.id, vertice_list[adj.id].weight), transpose_visited, g);
		}
	}
}

/*
 * Computes a topological ordering of the vertices.
 * For every vertex u in the order, and any of its
 * neighbours v, v appears later in the order than u.
 * You will be given a DAG as the argument.
 */

template <typename T>
vector<vertex<T>> topological_sort(directed_graph<T> g) {
  vector<vertex<T>> vertice_list = g.get_vertices();
  vector<bool> visited(vertice_list.size(), false); // initialise all visited to false  
  vector<vertex<T>> topo_result; // topological_sort result

  // iterate to all the vertices that is adjacent to v
  for(vertex<T> v : vertice_list){
	  if(!visited[v.id]){
	  	 visit(v, visited, g);
	  }
  }
  
  // unload the stack and push it to the final result
  while(!topo_stack.empty()){
	  topo_result.push_back(topo_stack.top());
	  topo_stack.pop();
  }

  return topo_result;
}

template <typename T>
void visit(vertex<T> v, vector<bool> &visited, directed_graph<T> g){
	map<int, map<int, int> > matrix; // matrix to determine which vertex are connected by edges`
    vector<vertex<T>> vertice_list = g.get_vertices();

	visited[v.id] = true;

	// initialise matrix for this graph
	for(edge<T> e: g.get_edges()){
		matrix[e.first][e.second] = 1;
	}

	// iterate to vertices adjacent to v
	for(vertex<T> adj : vertice_list){
		if(matrix[v.id][adj.id] == 1 && !visited[adj.id]){
			visit(vertex<T>(adj.id, vertice_list[adj.id].weight), visited, g);
		}
	}

	topo_stack.push(v);
}

/*
 * Computes the lowest cost-per-person for delivery over the graph.
 * u is the source vertex, which send deliveries to all other vertices.
 * vertices denote cities; vertex weights denote cities' population;
 * edge weights denote the fixed delivery cost between cities, which is irrelevant to 
 * the amount of goods being delivered. 
 */
template <typename T>
T low_cost_delivery(directed_graph<T> g, int u_id) {
	/*
	vector<vertex<T>> vertice_list = g.get_vertices();
	map<int, map<int, int> > matrix; // matrix to determine which vertex are connected by edges`
	vector<vertex<T>> shortest_path_list; // to contain the shortest path from u_id to other vertex
	
	int visited[vertice_list.size()][vertice_list.size()]; // fixed cost (check description)
	int total_edge = 0; // total of the edge weight
	int total_weight = 0; // total of the vertex weight
	int result = 0; // final result

	// initialise matrix
	for(edge<T> e: g.get_edges()){
		matrix[e.first][e.second] = e.weight;
		visited[e.first][e.second] = false;
	}

	for(vertex<T> v : vertice_list){
		// don't add the weight of u_id
		if(v.id != u_id){
		   total_weight += v.weight;
		}

		each_path.clear(); // global variables need to be reinitialised
		all_paths.clear(); // global variables need to be reinitialised

		// get the shortest_path of u_id to v.id
		shortest_path_list = shortest_path(g, u_id, v.id);

		// iterate on all the shortest_path_list add the edge and mark it as visited
		 for(int j = 0; j < shortest_path_list.size()-1; j++){

			int first = shortest_path_list[j].id;
			int second = shortest_path_list[j+1].id;

			// if not visited yet (based on the case where ypou can deliver at a fixed cost)
			if(!visited[first][second]){
				total_edge += matrix[first][second];
				visited[first][second] = true;
			}
		}
	}

	result = total_edge / total_weight;
	return result;
	*/
	return 0;
}
