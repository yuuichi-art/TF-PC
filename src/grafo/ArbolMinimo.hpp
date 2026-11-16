#ifndef GEODRONE_ARBOLMINIMO_HPP
#define GEODRONE_ARBOLMINIMO_HPP

#include "../geometria/Punto.hpp"   // INF

#include <vector>

namespace geodrone {

/**
 * @brief Árbol de expansión mínima de un subconjunto de nodos.
 * parent[i] es la posición (dentro de nodes) del padre de nodes[i]; la raíz nodes[0] tiene -1.
 */
struct MstResult {
    double cost = 0.0;          // Suma de las aristas del árbol
    double maxEdge = 0.0;       // Arista más larga del árbol
    std::vector<int> parent;
    bool spansAll = true;       // false si algún nodo no tiene camino (distancia infinita)
};

/**
 * @brief Algoritmo de Prim sobre una matriz de distancias completa, restringido a `nodes`.
 *
 * En cada paso agrega el nodo fuera del árbol más cercano a él y actualiza las distancias
 * de los demás. Con matriz (grafo completo) no hace falta cola de prioridad.
 *
 * Uso: red mínima de soporte entre estaciones (RF12), cota inferior del branch and bound
 * y base de la 2-aproximación.
 * Complejidad: O(k^2) en tiempo y O(k) en espacio, con k = nodes.size().
 * La arista más larga del árbol es además la menor autonomía con la que todos los nodos
 * quedan conectados con saltos que no la superan.
 */
inline MstResult primMst(const std::vector<std::vector<double>>& dist, const std::vector<int>& nodes) {
    int k = static_cast<int>(nodes.size());
    MstResult result;
    result.parent.assign(k, -1);
    if (k == 0) return result;

    std::vector<char> inTree(k, 0);
    std::vector<double> best(k, INF);   // best[i]: arista más corta de nodes[i] hacia el árbol
    best[0] = 0.0;

    for (int step = 0; step < k; ++step) {
        int next = -1;
        for (int i = 0; i < k; ++i) {
            if (!inTree[i] && (next == -1 || best[i] < best[next])) next = i;
        }
        if (best[next] == INF) {
            result.spansAll = false;    // Los nodos restantes no tienen camino al árbol
            break;
        }
        inTree[next] = 1;
        result.cost += best[next];
        if (best[next] > result.maxEdge) result.maxEdge = best[next];

        for (int i = 0; i < k; ++i) {
            double w = dist[nodes[next]][nodes[i]];
            if (!inTree[i] && w < best[i]) {
                best[i] = w;
                result.parent[i] = next;
            }
        }
    }
    return result;
}

} // namespace geodrone

#endif // GEODRONE_ARBOLMINIMO_HPP
