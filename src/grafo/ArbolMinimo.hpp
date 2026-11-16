#ifndef GEODRONE_ARBOLMINIMO_HPP
#define GEODRONE_ARBOLMINIMO_HPP

#include "../geometria/Punto.hpp"   // INF

#include <vector>

namespace geodrone {

/**
 * @brief Árbol de expansión mínima de un subconjunto de nodos.
 * padre[i] es la posición (dentro de nodos) del padre de nodos[i]; la raíz nodos[0] tiene -1.
 */
struct ArbolMinimo {
    double costo = 0.0;          // Suma de las aristas del árbol
    double aristaMaxima = 0.0;   // Arista más larga del árbol
    std::vector<int> padre;
    bool cubreTodos = true;       // false si algún nodo no tiene camino (distancia infinita)
};

/**
 * @brief Algoritmo de Prim sobre una matriz de distancias completa, restringido a `nodos`.
 *
 * En cada paso agrega el nodo fuera del árbol más cercano a él y actualiza las distancias
 * de los demás. Con matriz (grafo completo) no hace falta cola de prioridad.
 *
 * Uso: red mínima de soporte entre estaciones (RF12), cota inferior del branch and bound
 * y base de la 2-aproximación.
 * Complejidad: O(k^2) en tiempo y O(k) en espacio, con k = nodos.size().
 * La arista más larga del árbol es además la menor autonomía con la que todos los nodos
 * quedan conectados con saltos que no la superan.
 */
inline ArbolMinimo arbolMinimoPrim(const std::vector<std::vector<double>>& distancias, const std::vector<int>& nodos) {
    int k = static_cast<int>(nodos.size());
    ArbolMinimo resultado;
    resultado.padre.assign(k, -1);
    if (k == 0) return resultado;

    std::vector<char> enArbol(k, 0);
    std::vector<double> mejor(k, INF);   // mejor[i]: arista más corta de nodos[i] hacia el árbol
    mejor[0] = 0.0;

    for (int paso = 0; paso < k; ++paso) {
        int siguiente = -1;
        for (int i = 0; i < k; ++i) {
            if (!enArbol[i] && (siguiente == -1 || mejor[i] < mejor[siguiente])) siguiente = i;
        }
        if (mejor[siguiente] == INF) {
            resultado.cubreTodos = false;    // Los nodos restantes no tienen camino al árbol
            break;
        }
        enArbol[siguiente] = 1;
        resultado.costo += mejor[siguiente];
        if (mejor[siguiente] > resultado.aristaMaxima) resultado.aristaMaxima = mejor[siguiente];

        for (int i = 0; i < k; ++i) {
            double w = distancias[nodos[siguiente]][nodos[i]];
            if (!enArbol[i] && w < mejor[i]) {
                mejor[i] = w;
                resultado.padre[i] = siguiente;
            }
        }
    }
    return resultado;
}

} // namespace geodrone

#endif // GEODRONE_ARBOLMINIMO_HPP
