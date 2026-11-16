#ifndef GEODRONE_DIJKSTRA_HPP
#define GEODRONE_DIJKSTRA_HPP

#include "GrafoVisibilidad.hpp"
#include <queue>
#include <vector>

namespace geodrone {

/**
 * @brief Resultado de Dijkstra desde un origen: distancia mínima a cada nodo (INF si no se
 * alcanza) y su predecesor en el camino mínimo (-1 para el origen y los no alcanzados).
 */
struct ResultadoDijkstra {
    int origen;
    std::vector<double> distancia;
    std::vector<int> padre;

    ResultadoDijkstra(int n = 0, int inicio = -1)
        : origen(inicio), distancia(n, INF), padre(n, -1) {}
};

/**
 * @brief Algoritmo de Dijkstra con cola de prioridad (std::priority_queue).
 * Complejidad: O(|E| log |V|) por origen.
 */
class Dijkstra {
public:
    /**
     * @brief Calcula las distancias mínimas y predecesores desde un nodo origen `origen`
     * a todos los vértices del grafo de visibilidad.
     * @param nodosClaveComoExtremos Si es true, la base, las estaciones y los clientes (salvo el
     *        origen) se alcanzan pero no se expanden. Las distancias no cambian: un camino mínimo
     *        entre obstáculos solo dobla en vértices de obstáculos, porque doblar en otro punto
     *        se puede acortar. Así se ahorra trabajo y los caminos no cruzan otros clientes.
     */
    static ResultadoDijkstra ejecutar(const GrafoVisibilidad& grafo, int origen, bool nodosClaveComoExtremos = false) {
        int n = grafo.numNodos();
        ResultadoDijkstra resultado(n, origen);
        if (origen < 0 || origen >= n) return resultado;

        // Cola de mínimos ordenada por distancia acumulada: pair<distancia, nodo>
        using ElementoCola = std::pair<double, int>;
        std::priority_queue<ElementoCola, std::vector<ElementoCola>, std::greater<ElementoCola>> cola;

        resultado.distancia[origen] = 0.0;
        cola.emplace(0.0, origen);

        const auto& adyacencia = grafo.adyacencia();

        while (!cola.empty()) {
            auto [d, u] = cola.top();
            cola.pop();

            // Entrada vieja: el nodo ya se sacó antes con una distancia menor
            if (d > resultado.distancia[u] + EPS) {
                continue;
            }
            if (nodosClaveComoExtremos && u != origen && grafo.nodo(u).esNodoClave()) {
                continue;
            }

            for (const auto& arista : adyacencia[u]) {
                int v = arista.destino;
                double w = arista.peso;
                if (resultado.distancia[u] + w < resultado.distancia[v] - EPS) {
                    resultado.distancia[v] = resultado.distancia[u] + w;
                    resultado.padre[v] = u;
                    cola.emplace(resultado.distancia[v], v);
                }
            }
        }

        return resultado;
    }
};

} // namespace geodrone

#endif // GEODRONE_DIJKSTRA_HPP
