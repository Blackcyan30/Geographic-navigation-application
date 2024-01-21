// application.cpp
// <Talha Adnan>
// Environment: Visual Studio Code.
// This file contains most of the implementation of the google maps back end replica application. Mostly within the application function.
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <stdexcept>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;

struct prioritize{ // Priority condition for priority queue.
  bool operator()(const pair<long long, double>& p1, const pair<long long, double>&p2) const{
    return p1.second > p2.second;
  }
};
const double INF = numeric_limits<double>::max();

void dijkstraShortestPath(const graph<long long, double>& G, map<long long, double>& dist, map<long long, long long>& prev, long long startIndexKey);

BuildingInfo searchBuilding(string query, const vector<BuildingInfo> &Buildings, int &errorState);

Coordinates getNearestNodeToBuilding(const BuildingInfo &building, vector<FootwayInfo>& Footways, map<long long, Coordinates>& Nodes);

vector <long long> getPath(const map<long long, long long>& prev, long long endNode);

void printPath(vector<long long> path);

void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double> G);

int main() {

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  graph <long long, double> G;

  // MileStone 5: Adding vertices to the graph.
  for (const auto &e : Nodes){
    G.addVertex(e.first);
  }

  // Milestone 6: Adding edges.
  // example: get the latitude of the second node of the fifth footway info:
  //                       double latitude = Nodes[Footways[4].Nodes[1]].Lat
  
  for (const auto &footWay : Footways){
    for(size_t i = 0; i < footWay.Nodes.size() - 1; i++){
        double dist = distBetween2Points(Nodes[footWay.Nodes.at(i)].Lat, Nodes[footWay.Nodes.at(i)].Lon, 
                              Nodes[footWay.Nodes.at(i + 1)].Lat,Nodes[footWay.Nodes.at(i + 1)].Lon);
        G.addEdge(footWay.Nodes.at(i), footWay.Nodes.at(i+1), dist);
        G.addEdge(footWay.Nodes.at(i+1), footWay.Nodes.at(i), dist);
    }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}

/// @brief This function contains the working of my standard application.
/// @param Nodes 
/// @param Footways 
/// @param Buildings 
/// @param G 
void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double> G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);
    
    // Milestone 7: Search buildings 1 and 2.
    // This is done by calling the searchBuilding function to determine the closest match to the userinputed builing name.
    int searchBuildingErrorState;
    BuildingInfo building1 = searchBuilding(person1Building, Buildings, searchBuildingErrorState);
    BuildingInfo building2 = searchBuilding(person2Building, Buildings, searchBuildingErrorState);
   
    // Checking if building is found or not. If building is found we go into the else statement.
    if (building1.Fullname == ""){
      cout << "Person 1's building not found" << endl;

    }else if(building2.Fullname == ""){
      cout << "Person 2's building not found" << endl;

    }else{
      
      // Milestone 8: locate center building.
      Coordinates midPoint = centerBetween2Points(building1.Coords.Lat, building1.Coords.Lon, building2.Coords.Lat, building2.Coords.Lon);

      // Finding nearest building to the previously calculated midpoint. 
      // This is done by using a standard min finding algorithm to get the building that has the lowest dist from the midpoint.
      /*
      Steps: 
      1. Loop through the buildings vector.
      2. Calc the dist using the distBetween2Points which is a func that takes in lat and long.
      3. Update the minDist var with the dist calculated if dist calculated is less than the current value in the minDist var; 
         with that also update the minDistBuilding var to the current building where the min dist was found.
      5. Do this untill the end of the vector is reached and then we will have the closest building in the minDistBuilding var.
      */
      BuildingInfo buildingCenter;
      double minDist = INF; // Initialized to infinity.
      for (const auto &building : Buildings){
        double dist = distBetween2Points(midPoint.Lat, midPoint.Lon, building.Coords.Lat, building.Coords.Lon);
        if (dist < minDist){
          minDist = dist;
          buildingCenter = building;
        }
      }

      //
      // MILESTONE 9: Find Nearest Nodes from buildings 1, 2 & Center
      //
      Coordinates nearestNodeB1 = getNearestNodeToBuilding(building1, Footways, Nodes);
      Coordinates nearestNodeB2 = getNearestNodeToBuilding(building2, Footways, Nodes);
      Coordinates nearestNodeCenter =  getNearestNodeToBuilding(buildingCenter, Footways, Nodes);

      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << building1.Fullname << endl;
      cout << "(" << building1.Coords.Lat << ", " << building1.Coords.Lon << ")" << endl;

      cout << "Person 2's point:" << endl;
      cout << " " << building2.Fullname << endl;
      cout << "(" << building2.Coords.Lat << ", " << building2.Coords.Lon << ")" << endl;

      cout << "Destination Building:" << endl;
      cout << " " << buildingCenter.Fullname << endl;
      cout << "(" << buildingCenter.Coords.Lat << ", " << buildingCenter.Coords.Lon << ")" << endl;

      cout << "Nearest P1 node:" << endl;
      cout << " " << nearestNodeB1.ID << endl;
      cout << " " << "(" << nearestNodeB1.Lat << ", " << nearestNodeB1.Lon << ")" << endl;
      cout << "Nearest P2 node:" << endl;
      cout << " " << nearestNodeB2.ID << endl;
      cout << " " << "(" << nearestNodeB2.Lat << ", " << nearestNodeB2.Lon << ")" << endl;
      cout << "Nearest destination node:" << endl;
      cout << " " << nearestNodeCenter.ID << endl;
      cout << " " << "(" << nearestNodeCenter.Lat << ", " << nearestNodeCenter.Lon << ")" << endl;
      //
      // MILESTONE 10: Run Dijkstra’s Algorithm
      //
      map<long long, double> dist1; map<long long, long long> prev1; 
      map<long long, double> dist2; map<long long, long long> prev2;
      map<long long, double> distCenter; map<long long, long long> prevCenter;
      
      // Here I lookup buildings, find nearest start and dest nodes, find center
      // run Dijkstra's alg from each start, output distances and paths to destination:
    
      dijkstraShortestPath(G, dist1, prev1, nearestNodeB1.ID);

      dijkstraShortestPath(G, dist2, prev2, nearestNodeB2.ID);
 
      //
      // MILESTONE 11: Print path (path found! break)
      //
      if (dist1[nearestNodeB2.ID] >= INF){
        cout << "Sorry, destination unreachable." << endl;
      }
      else if (dist1[nearestNodeCenter.ID] >= INF || dist2[nearestNodeCenter.ID] >= INF){
        cout << "At least one person was unable to reach the destination building. Finding next closest building..." << endl;
      }else{
    
        vector<long long> path1 = getPath(prev1, nearestNodeCenter.ID);
        
        vector<long long> path2 = getPath(prev2, nearestNodeCenter.ID);
        cout << "Person 1's distance to dest: " << dist1[nearestNodeCenter.ID] << " miles" << endl;
        cout << "Path: "; printPath(path1);
        
        cout << "Person 2's distance to dest: " << dist2[nearestNodeCenter.ID] << " miles" << endl;
        cout << "Path: "; printPath(path2);

        // MILESTONE 11: Find Second Nearest Destination (loop again)
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}

/// @brief Returns the closest match to a building for a given search query.
/// @param query 
/// @param Buildings 
/// @return BuildingInfo
BuildingInfo searchBuilding(string query, const vector<BuildingInfo> &Buildings, int &errorState){

  errorState = 0;
  // First searching for abbreviations
  for (const auto &building : Buildings){
    if (building.Abbrev == query){
      return building;
    }
  }

  // Then searching for partial names and matches in the Fullname subitem of BuildingInfo
  for (const auto &building : Buildings){
    if (building.Fullname.find(query) != string::npos){
      return building;
    }
  }

  errorState = -1;
  return BuildingInfo();
}

/// @brief This function returns the key of the nearest node distance wise to the given building using standard min finding algorithm.
/// @param building 
/// @param Nodes 
/// @return Key of the nearest node.
Coordinates getNearestNodeToBuilding(const BuildingInfo &building, vector<FootwayInfo>& Footways, map<long long, Coordinates>& Nodes){

  Coordinates nearestNodeCoord;
  double minDist = INF; // Initilized to infinity then updated once new lowest value is found
  
  for (const auto &footWay : Footways){
    for(size_t i = 0; i < footWay.Nodes.size(); i++){
        double dist = distBetween2Points(Nodes[footWay.Nodes.at(i)].Lat, Nodes[footWay.Nodes.at(i)].Lon, building.Coords.Lat, building.Coords.Lon);
        if (dist < minDist){
          minDist = dist;
          nearestNodeCoord = Nodes[footWay.Nodes.at(i)];
        }
    }
  }

  return nearestNodeCoord;
}

/// @brief This function is a standard implementation of Dijkstra's Algorithm for a shortest path.
/// @details This function implements Dijkstra's Algorithm to find the shortest path in a graph implicitly returns the prev map which is mapping of the predecessors of every vertex found using the algorithm.
///          It also implicitly returns a mapping of the distance needed to reach every node reachable from the start point. This implimentation is more commonly known as the lazy implementation of Dijkstra's Algorithm.
/// @param G 
/// @param dist 
/// @param prev 
/// @param startIndexKey 
void dijkstraShortestPath(const graph<long long, double>& G, map<long long, double>& dist, map<long long, long long>& prev, long long startIndexKey){
  priority_queue<pair<long long, double>, vector<pair<long long, double> >, prioritize> pq; 

  // Initializing the dist to infinity and prev to 0
  for (const auto& currentVertex: G.getVertices()){
    dist[currentVertex] = INF;
    prev[currentVertex] = 0;
  }

  // Pushing the start node inside the priority queue with it's dist to be 0 and also updating the distance of the start node to be 0 in the dist array.
  dist[startIndexKey] = 0;
  pq.push(make_pair(startIndexKey, 0));

  while (!pq.empty()) {
    long long currentVertex = pq.top().first; // Getting the value of the first item in the priority queue.
    pq.pop(); // Popping the first item out of the priority queue.
    for (const auto &vertex : G.neighbors(currentVertex)) { // Searching all the neighbors of the start vertex of the neighbor with the least dist required to go to it.
      
      // Calculation of distance from previous vertex to the new vertex.
      double weight = 0;
      G.getWeight(currentVertex, vertex, weight);
      double newDist = dist[currentVertex] + weight;

      if (newDist < dist[vertex]) { // If a new dist is found and is lower then the one in the already in the dist array then, add that in the dist map and update the prev map to with the vert and the current vert to be it's predesessor, while also pushing the newly found node witht he least weight in the priority queue to be searched next. This happens due to the greedy nature of Dijkstra's algorithm.
        dist[vertex] = newDist;
        prev[vertex] = currentVertex;
        pq.push(make_pair(vertex, newDist));
      }
    }
  }

}

//TODO:

/// @brief This function outputs the path to any given node with the prev array of the start node.
/// @param prev 
/// @param endNode 
/// @return vector of the path to the endNode.
vector <long long> getPath(const map<long long, long long>& prev, long long endNode){

  vector <long long> v;

  while(endNode != 0){
    v.push_back(endNode);
    endNode = prev.at(endNode);
  }

  reverse(v.begin(), v.end());
  return v;
}

/// @brief this function prints out the path in the reqired format.
/// @param path 
void printPath(vector<long long> path){
  for (size_t i = 0; i < path.size(); i++){
    if (i == path.size() - 1)
      cout << path.at(i) << endl;
    else  
      cout << path.at(i) << "->";
  }
}