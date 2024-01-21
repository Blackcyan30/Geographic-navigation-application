#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "graph.h"
#include <map>
#include <vector>

using namespace std;

TEST(Graph, addVertex){
    graph<string, int> g;

    EXPECT_TRUE(g.addVertex("A"));
    EXPECT_TRUE(g.addVertex("B"));
    EXPECT_FALSE(g.addVertex("A"));  // Adding the same vertex again should return false
    EXPECT_EQ(g.NumVertices(), 2);


}

TEST(Graph, addEdge){
    graph<string, int> g;

    g.addVertex("A");
    g.addVertex("B");
    g.addVertex("C");

    // Test 1: Add a new edge
    EXPECT_TRUE(g.addEdge("A", "B", 5));

    // Test 2: Add an edge with the same vertices but a different weight, and check if it updates the weight
    EXPECT_TRUE(g.addEdge("A", "B", 8));
    int weight;
    EXPECT_TRUE(g.getWeight("A", "B", weight));
    EXPECT_EQ(weight, 8);

    // Test 3: Attempt to add an edge with non-existent 'from' vertex, and check if it returns false
    EXPECT_FALSE(g.addEdge("D", "B", 3));

    // Test 4: Attempt to add an edge with non-existent 'to' vertex, and check if it returns false
    EXPECT_FALSE(g.addEdge("A", "D", 2));

    // Total number of edges should be 1 (first edge) + 1 (updated weight)
    EXPECT_EQ(g.NumEdges(), 1);

}

TEST(Graph, getWeight) {
    // Test case 1: Edge exists
    graph<string, int> g1;
    g1.addVertex("A");
    g1.addVertex("B");
    g1.addEdge("A", "B", 5);

    int weight1;
    EXPECT_TRUE(g1.getWeight("A", "B", weight1));
    EXPECT_EQ(weight1, 5);

    // Test case 2: Edge does not exist
    graph<string, int> g2;
    g2.addVertex("A");
    g2.addVertex("B");

    int weight2 = 0;
    EXPECT_FALSE(g2.getWeight("A", "B", weight2));
    EXPECT_EQ(weight2, 0);

    // Test case 3: Invalid 'from' vertex
    graph<string, int> g3;
    g3.addVertex("A");
    g3.addVertex("B");
    g3.addEdge("A", "B", 8);

    int weight3 = 0;
    EXPECT_FALSE(g3.getWeight("C", "B", weight3));  // 'C' is not a valid 'from' vertex
    EXPECT_EQ(weight3, 0);

    // Test case 4: Invalid 'to' vertex
    graph<string, int> g4;
    g4.addVertex("A");
    g4.addVertex("B");
    g4.addEdge("A", "B", 8);

    int weight4 = 0;
    EXPECT_FALSE(g4.getWeight("A", "C", weight4));  // 'C' is not a valid 'to' vertex
    EXPECT_EQ(weight4, 0);
}

TEST(Graph, Neighbors) {
    graph<string, int> g;

    // Test 1: Add vertices and edges
    g.addVertex("A");
    g.addVertex("B");
    g.addVertex("C");
    g.addVertex("D");
    g.addEdge("A", "B", 5);
    g.addEdge("A", "C", 8);
    g.addEdge("B", "D", 3);

    // Test 2: Check neighbors for an existing vertex
    set<string> neighborsA = g.neighbors("A");
    set<string> expectedNeighborsA = {"B", "C"};
    EXPECT_EQ(neighborsA, expectedNeighborsA);

    // Test 3: Check neighbors for a vertex with no outgoing edges
    set<string> neighborsD = g.neighbors("D");
    set<string> expectedNeighborsD = {};
    EXPECT_EQ(neighborsD, expectedNeighborsD);

    // Test 4: Check neighbors for a non-existent vertex
    set<string> neighborsX = g.neighbors("X");
    set<string> expectedNeighborsX = {};
    EXPECT_EQ(neighborsX, expectedNeighborsX);

    // Test 5: Add more vertices and edges
    g.addVertex("E");
    g.addEdge("D", "E", 2);

    // Test 6: Check neighbors after adding more vertices and edges
    set<string> neighborsB = g.neighbors("B");
    set<string> expectedNeighborsB = {"D"};
    EXPECT_EQ(neighborsB, expectedNeighborsB);

    // Test 7: Check neighbors for a vertex with incoming edges
    set<string> neighborsC = g.neighbors("C");
    set<string> expectedNeighborsC = {};
    EXPECT_EQ(neighborsC, expectedNeighborsC);

    // Test 8: Check neighbors for a vertex which does not exist.
    set<string> neighborsE = g.neighbors("E");
    set<string> expectedNeighborsE = {};
    EXPECT_EQ(neighborsE, expectedNeighborsE);
}