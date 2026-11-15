#include "MissionGraph.hpp"
#include "Dijkstra.hpp"

#include <algorithm>
#include <string>

namespace geodrone {

MissionGraph::MissionGraph(const Scenario& scenario)
    : numStations_(scenario.numStations()), numClients_(scenario.numClients()) {
    // Primero los nodos clave, en el orden base, estaciones, clientes; después los obstáculos.
    graph_.setDepot(scenario.depot, "D0");
    for (int i = 0; i < numStations_; ++i) {
        graph_.addChargingStation(scenario.stations[i], "S" + std::to_string(i + 1));
    }
    for (int i = 0; i < numClients_; ++i) {
        graph_.addClient(scenario.clients[i], "C" + std::to_string(i + 1));
    }
    for (const Polygon& poly : scenario.obstacles) {
        graph_.addObstacle(poly);
    }
    graph_.buildVisibilityEdges();

    // Un Dijkstra por nodo clave: da una fila de la matriz y los predecesores para
    // reconstruir los caminos reales desde ese nodo.
    int keyCount = numKeyNodes();
    dist_.assign(keyCount, std::vector<double>(keyCount, Dijkstra::INF));
    parent_.resize(keyCount);
    for (int a = 0; a < keyCount; ++a) {
        DijkstraResult result = Dijkstra::run(graph_, a, true);
        for (int b = 0; b < keyCount; ++b) {
            dist_[a][b] = result.dist[b];
        }
        parent_[a] = std::move(result.parent);
    }

    connected_.assign(graph_.numVertices(), 0);
    markConnected(depotKey());
}

// DFS recursivo: marca todo lo alcanzable desde node. Cada nodo se visita una vez y cada
// arista se revisa dos veces: O(V + E). La profundidad máxima es V (unos pocos miles).
void MissionGraph::markConnected(int node) {
    connected_[node] = 1;
    for (const GraphEdge& edge : graph_.getNeighbors(node)) {
        if (!connected_[edge.to]) {
            markConnected(edge.to);
        }
    }
}

std::vector<int> MissionGraph::unreachableClients() const {
    std::vector<int> clients;
    for (int i = 0; i < numClients_; ++i) {
        if (!isConnectedToDepot(clientKey(i))) {
            clients.push_back(i);
        }
    }
    return clients;
}

std::vector<int> MissionGraph::pathNodes(int fromKey, int toKey) const {
    std::vector<int> path;
    if (dist_[fromKey][toKey] == Dijkstra::INF) return path;

    for (int node = toKey; node != -1; node = parent_[fromKey][node]) {
        path.push_back(node);
        if (node == fromKey) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace geodrone
