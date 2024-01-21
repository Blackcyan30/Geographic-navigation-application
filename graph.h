// < Talha Adnan >
// This file contains the implementation of the graph class.


#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <map>

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
  struct edge {
    VertexT vertexTo;
    WeightT  dist;

    edge(VertexT v, WeightT  d){
      vertexTo = v;
      dist = d;
    }

  };

  map<VertexT, vector<edge> > adjacencyList;

  /// @brief Finds the vertex in the Vertices vector and returns it's index position if found, otherwise returns -1.
  /// @param v 
  /// @return int.
  int _LookupVertex(VertexT v) const {
    int exists = adjacencyList.count(v);
    // if get here, not found:
    return exists;
  }

  /// @brief Private helper function that helps finding a edge in a vector of edges.
  /// @param v 
  /// @param e 
  /// @return index where edge is found.
  int findEdge(const vector<edge>& v, VertexT e) const {

    // Loop through the vector and see if we can find the vertex
    for (size_t i = 0; i < v.size(); i++){
      if (e == v.at(i).vertexTo){
        return i;
      }
    }
    
    // If we reach here that mean edge is not found.
    return -1;
  }

 public:

  /// @brief Default constructor for graph
  /// @details Constructs an empty graph without any arguments. The constructor initializes the graph with no vertices or edges, 
  ///          and it relies on other member functions (such as addVertex and addEdge) to populate the graph.
  graph() {}

  /// @brief Returns the number of vertices currently in the graph
  /// @return 
  int NumVertices() const {
    return static_cast<int>(this->adjacencyList.size());
  }
  
  /// @brief Returns the number of edges currently in the graph.
  /// @return integer.
  int NumEdges() const {
    int count = 0; // Initializing count to zero.

    // Loops through the adjecenyList, taking the size of the number of edges going out of each vertex. 
    for (const auto &[k,v] : adjacencyList){
      count += v.size();
    }
    return count;
  }

  /// @brief Adds vertex v to the graph. If the vertex already exists, false is returned.
  /// @param v 
  /// @return 
  bool addVertex(VertexT v) {

    // Checking if vertex is already in graph? If so do not insert and return false.
    if (_LookupVertex(v) == 1) {
      return false;
    }
    // If we get here, vertex does not exist so insert in the adjacencyList.
    adjacencyList[v];

    return true;
  }

  /// @brief Adds edge(from, to , weight) to the graph, and returns true if successfull.
  /// @details Adds the edge (from, to, weight) to the graph, and returns true. 
  ///          If the edge already exists, the existing edge weight is overwritten with the new edge weight.
  ///          If the vertices do not exist, false is returned. 
  /// @note First check if the edge to build already exists in the adjecency list. 
  ///       If so then update it's edge weight with the new weight. If not then 
  ///       make a new edge.
  /// @param from 
  /// @param to 
  /// @param weight 
  /// @return boolean 
  bool addEdge(VertexT from, VertexT to, WeightT weight) {

    // Checking to see if vertices exist.
    if (_LookupVertex(from) == 0 || _LookupVertex(to) == 0) return false;

    // Creating new Edge to add.
    edge newEdge(to, weight);
    int found = findEdge(adjacencyList[from], to);
    if(found >= 0){ 
    // findEdge returns -1 if not found. If found returns the index of the pos found at.
      adjacencyList[from].at(found).dist = weight;
    }
    else{
      adjacencyList[from].push_back(newEdge);
    }

    return true;
  }

  /// @brief getWeight returns the weight associated with the given edge.
  /// @details If the edge exists, the weight is returned via the reference parameter and true is returned.
  ///          If the edge does not exist, the weight parameter is unchanged and false is returned.
  /// @param from 
  /// @param to 
  /// @param weight 
  /// @return 
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    
    if (_LookupVertex(from) == 0 || _LookupVertex(to) == 0) return false;
    
    int found = findEdge(adjacencyList.at(from), to);

    if (found >= 0){
      weight = adjacencyList.at(from).at(found).dist;
      return true;
    }

    return false;
  }

  /// @brief Returns a set containing all neighbors of v.
  /// @details Gives all vertices that can be reached from v along one edge.
  ///          Since a set is returned, the neighbors are returned in sorted order.
  /// @note Use foreach to iterate through the set.
  /// @param v 
  /// @return set of VertexT.
  set<VertexT> neighbors(VertexT v) const{
    set<VertexT>  S;

    // We search the Vertices vector and see if the vertex exists or not.
    if (_LookupVertex(v) == 0) return S; // Vertex does not exist return empty set.

    // We know that vertex exists so now we go to the adjecency list and 
    // using a foreach loop get every edge and add it's vertexTo sub-item in the set i.e. vertex
    for (const auto edge : adjacencyList.at(v)){
      S.emplace(edge.vertexTo);
    }

    return S;
  }

  /// @brief Returns a vector containing all the vertices currently in the graph.
  /// @return vector <VertexT> 
  vector<VertexT> getVertices() const {
    vector<VertexT> v;
    for (const auto &[k,val] : adjacencyList){
      v.push_back(k);
    }
    return v;  // returns a copy:
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //


// This function is commented out for testing purposes.
  
  /// @brief Dumps the internal stae of the graph for debugging purposes.
  /// @param output 
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    for (int i = 0; i < this->NumVertices(); ++i) {
      output << " " << i << ". " << this->Vertices[i] << endl;
    }

    output << endl;
    output << "AdjacencyList:" << endl;
    for (const auto &v : getVertices()){
      output << "Vertex: "<< v << ": ";
      set<string> neighbors = this->neighbors(v);
      for (const auto &edge : neighbors){
        int weight = 0;
        getWeight(v, edge, weight); 
        output << "(" << edge << ", " << weight << ") ";
      }
      output << endl;
      
    }

    output << "**************************************************" << endl;
  }
};
