#ifndef GEODRONE_DIJKSTRA_HPP
#define GEODRONE_DIJKSTRA_HPP

#include "VisibilityGraph.hpp"
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

namespace geodrone {

/**
 * @brief Resultado de una ejecución de Dijkstra de fuente única (SSSP).
 */
struct DijkstraResult {
    int source;
    std::vector<double> dist;
    std::vector<int> parent;

    static constexpr double INF = std::numeric_limits<double>::infinity();

    DijkstraResult(int n = 0, int src = -1)
        : source(src), dist(n, INF), parent(n, -1) {}

    bool isReachable(int target) const {
        return target >= 0 && target < static_cast<int>(dist.size()) && dist[target] < INF / 2.0;
    }

    /**
     * @brief Reconstruye la secuencia de vértices del camino mínimo desde source hasta target.
     */
    std::vector<int> getPathTo(int target) const {
        std::vector<int> path;
        if (!isReachable(target)) return path;

        for (int curr = target; curr != -1; curr = parent[curr]) {
            path.push_back(curr);
            if (curr == source) break;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

/**
 * @brief Algoritmo de Dijkstra optimizado con cola de prioridad std::priority_queue.
 * Complejidad: O(|E| log |V|) por fuente.
 */
class Dijkstra {
public:
    static constexpr double INF = std::numeric_limits<double>::infinity();

    /**
     * @brief Calcula las distancias mínimas y predecesores desde un nodo origen `source`
     * a todos los vértices del grafo de visibilidad.
     */
    static DijkstraResult run(const VisibilityGraph& graph, int source) {
        int n = graph.numVertices();
        DijkstraResult res(n, source);
        if (source < 0 || source >= n) return res;

        // min-heap ordenado por distancia acumulada: pair<dist, vertex_idx>
        using QueueElement = std::pair<double, int>;
        std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;

        res.dist[source] = 0.0;
        pq.emplace(0.0, source);

        const auto& adj = graph.getAdj();

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();

            // Descartar entradas obsoletas (lazy deletion en heap)
            if (d > res.dist[u] + EPS) {
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

    /**
     * @brief Construye la Matriz de Distancias Reducida (Grafo de Misión GM) entre nodos clave K = {D} ∪ C ∪ S.
     * Fase 1 del desacoplamiento arquitectural.
     * Complejidad: O(|K| * |E| log |V|).
     * @param graph Grafo de visibilidad construido.
     * @param keyIndices Índices de los nodos clave a interconectar.
     * @return Matriz |K| x |K| con los costos geodésicos mínimos libres de colisiones.
     */
    static std::vector<std::vector<double>> buildReducedCostMatrix(
        const VisibilityGraph& graph,
        const std::vector<int>& keyIndices)
    {
        int k = static_cast<int>(keyIndices.size());
        std::vector<std::vector<double>> costMatrix(k, std::vector<double>(k, INF));

        for (int i = 0; i < k; ++i) {
            int srcNode = keyIndices[i];
            DijkstraResult sssp = run(graph, srcNode);

            for (int j = 0; j < k; ++j) {
                int targetNode = keyIndices[j];
                costMatrix[i][j] = sssp.dist[targetNode];
            }
        }

        return costMatrix;
    }
};

} // namespace geodrone

#endif // GEODRONE_DIJKSTRA_HPP
