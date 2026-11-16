#ifndef GEODRONE_GRAFOMISION_HPP
#define GEODRONE_GRAFOMISION_HPP

#include "GrafoVisibilidad.hpp"
#include "../escenario/Escenario.hpp"

#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Grafo de misión de un escenario: grafo de visibilidad, matriz de distancias entre
 * nodos clave, caminos reales entre ellos y conectividad con la base.
 *
 * Nodos clave (índice k): 0 = base, 1..S = estaciones, S+1..S+N = clientes. Se agregan al
 * grafo de visibilidad antes que los vértices de obstáculos, así que el nodo clave k es
 * también el nodo k del grafo.
 *
 * Construcción, con V nodos, E aristas, K nodos clave y M aristas de obstáculos:
 * - Grafo de visibilidad: O(V^2 · M) en el peor caso.
 * - Matriz: Dijkstra desde cada nodo clave, O(K · E log V) en tiempo.
 * - Conectividad: un DFS desde la base, O(V + E).
 * Espacio: O(V + E) del grafo, más O(K^2) de la matriz y O(K · V) de predecesores.
 */
class MissionGraph {
public:
    explicit MissionGraph(const Scenario& scenario);

    int numKeyNodes() const { return 1 + numStations_ + numClients_; }
    int numStations() const { return numStations_; }
    int numClients() const { return numClients_; }

    int depotKey() const { return 0; }
    int stationKey(int station) const { return 1 + station; }
    int clientKey(int client) const { return 1 + numStations_ + client; }

    const Point& keyPoint(int key) const { return graph_.getNode(key).pt; }

    // Etiqueta del nodo clave: "D0", "S1".."S<S>" o "C1".."C<N>", como en los archivos de ruta.
    const std::string& label(int key) const { return graph_.getNode(key).name; }

    // Distancia de vuelo mínima entre dos nodos clave (infinito si no hay camino).
    double distance(int fromKey, int toKey) const { return dist_[fromKey][toKey]; }
    const std::vector<std::vector<double>>& distances() const { return dist_; }

    // Resultado del DFS desde la base.
    bool isConnectedToDepot(int key) const { return connected_[key] != 0; }

    // Clientes (índices 0..N-1) que los obstáculos separan de la base.
    std::vector<int> unreachableClients() const;

    // Nodos del grafo de visibilidad que forma el camino mínimo entre dos nodos clave,
    // incluidos ambos extremos. Vacío si no hay camino.
    std::vector<int> pathNodes(int fromKey, int toKey) const;

    const VisibilityGraph& graph() const { return graph_; }

private:
    VisibilityGraph graph_;
    int numStations_ = 0;
    int numClients_ = 0;
    std::vector<std::vector<double>> dist_;     // dist_[a][b] entre nodos clave
    std::vector<std::vector<int>> parent_;      // parent_[a][v]: predecesor de v en el camino desde a
    std::vector<char> connected_;               // connected_[v]: v se alcanza desde la base

    void markConnected(int node);
};

} // namespace geodrone

#endif // GEODRONE_GRAFOMISION_HPP
