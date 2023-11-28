#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "GraphBase.hpp"
#include <unordered_map>
#include <list>
#include <string>
#include <vector>
#include <limits>
#include <set>

class Vertex;

class Edge {
public:

    //incident vertexes
    Vertex* vertex1;
    Vertex* vertex2;
    
    std::string neighborValue;
    unsigned long weight;

    //constructor
    Edge(Vertex* v1, Vertex* v2, unsigned long wt) : vertex1(v1), vertex2(v2), weight(wt) {}        
};

class Vertex {
public:

    std::string label;
    std::list<Edge*> adjList;

    //constructor
    Vertex(std::string lbl) : label(std::move(lbl)) {}
};

class Graph : public GraphBase {
private:
    std::unordered_map<std::string, Vertex*> vertices;
    std::list<Edge*> edges;

public:
    //constructor
    Graph();
    //destructor
    ~Graph();

    //manipulate graph
    void addVertex(std::string label) override;
    void removeVertex(std::string label) override;
    void addEdge(std::string label1, std::string label2, unsigned long weight) override;
    void removeEdge(std::string label1, std::string label2) override;

    //Dijkstra's Algorithm
    unsigned long shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path) override;
};

#endif