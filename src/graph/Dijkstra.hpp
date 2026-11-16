#ifndef GEODRONE_DIJKSTRA_HPP
#define GEODRONE_DIJKSTRA_HPP

#include "VisibilityGraph.hpp"
#include <queue>
#include <vector>

namespace geodrone {

/**
 * @brief Resultado de Dijkstra desde un origen: distancia mínima a cada nodo (INF si no se
 * alcanza) y su predecesor en el camino mínimo (-1 para el origen y los no alcanzados).
 */
struct DijkstraResult {
    int source;
    std::vector<double> dist;
    std::vector<int> parent;

    DijkstraResult(int n = 0, int src = -1)
        : source(src), dist(n, INF), parent(n, -1) {}
};

/**
 * @brief Algoritmo de Dijkstra con cola de prioridad (std::priority_queue).
 * Complejidad: O(|E| log |V|) por origen.
 */
class Dijkstra {
public:
    /**
     * @brief Calcula las distancias mínimas y predecesores desde un nodo origen `source`
     * a todos los vértices del grafo de visibilidad.
     * @param keyNodesAsEndpoints Si es true, la base, las estaciones y los clientes (salvo el
     *        origen) se alcanzan pero no se expanden. Las distancias no cambian: un camino mínimo
     *        entre obstáculos solo dobla en vértices de obstáculos, porque doblar en otro punto
     *        se puede acortar. Así se ahorra trabajo y los caminos no cruzan otros clientes.
     */
    static DijkstraResult run(const VisibilityGraph& graph, int source, bool keyNodesAsEndpoints = false) {
        int n = graph.numVertices();
        DijkstraResult res(n, source);
        if (source < 0 || source >= n) return res;

        // Cola de mínimos ordenada por distancia acumulada: pair<distancia, nodo>
        using QueueElement = std::pair<double, int>;
        std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;

        res.dist[source] = 0.0;
        pq.emplace(0.0, source);

        const auto& adj = graph.getAdj();

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();

            // Entrada vieja: el nodo ya se sacó antes con una distancia menor
            if (d > res.dist[u] + EPS) {
                continue;
            }
            if (keyNodesAsEndpoints && u != source && graph.getNode(u).isKeyNode()) {
                continue;
            }

            for (const auto& edge : adj[u]) {
                int v = edge.to;
                double w = edge.weight;
                if (res.dist[u] + w < res.dist[v] - EPS) {
                    res.dist[v] = res.dist[u] + w;
                    res.parent[v] = u;
                    pq.emplace(res.dist[v], v);
                }
            }
        }

        return res;
    }
};

} // namespace geodrone

#endif // GEODRONE_DIJKSTRA_HPP
