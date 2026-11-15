#ifndef GEODRONE_HEURISTICSOLVER_HPP
#define GEODRONE_HEURISTICSOLVER_HPP

#include <chrono>
#include <cstdint>
#include <vector>

namespace geodrone {

/**
 * @brief Heurísticas para Medium, Large y Extreme (RF08), sobre el orden de visita.
 *
 * Como el solver exacto, trabajan sobre la matriz de distancias del grafo de misión y
 * devuelven las claves de los clientes en orden de visita, sin la base; las recargas se
 * insertan después (decisión 5). Todas suponen que los clientes son alcanzables.
 *
 * - Constructivas: vecino más cercano (greedy) y 2-aproximación por árbol de expansión mínima.
 * - Mejora: 2-opt y búsqueda local iterada (aleatoria, con semilla).
 */
namespace HeuristicSolver {

    using Clock = std::chrono::steady_clock;

    // Sin límite de tiempo.
    inline Clock::time_point noDeadline() { return Clock::time_point::max(); }

    // Iteraciones por defecto de la búsqueda local iterada (fijas, para que sea reproducible).
    inline constexpr int DEFAULT_ILS_ITERATIONS = 200;

    /**
     * @brief Costo del ciclo base -> clientes en ese orden -> base. O(n).
     */
    double tourCost(const std::vector<std::vector<double>>& dist, int depot, const std::vector<int>& order);

    /**
     * @brief Vecino más cercano (greedy): desde la base, ir siempre al cliente pendiente más cercano.
     * Rápido pero sin garantía de calidad. Complejidad: O(n^2) en tiempo, O(n) en espacio.
     */
    std::vector<int> nearestNeighbor(const std::vector<std::vector<double>>& dist, int depot,
                                     const std::vector<int>& clients);

    /**
     * @brief 2-aproximación: árbol de expansión mínima (Prim) de la base y los clientes, recorrido
     * en preorden desde la base (DFS). El orden de visita es el de ese recorrido.
     *
     * Garantía: costo ≤ 2 · costo del árbol ≤ 2 · óptimo. Recorrer el árbol ida y vuelta cuesta
     * 2 · árbol; el preorden salta los nodos ya visitados, y esos atajos nunca alargan el camino
     * porque la matriz de caminos mínimos cumple la desigualdad triangular. Y el óptimo, sin una
     * arista, es un árbol, así que cuesta al menos lo que el árbol mínimo.
     * Complejidad: O(n^2) en tiempo (Prim sobre matriz) y O(n) en espacio.
     */
    std::vector<int> mstApproximation(const std::vector<std::vector<double>>& dist, int depot,
                                      const std::vector<int>& clients);

    /**
     * @brief 2-opt: mientras exista, aplica un intercambio que acorte la ruta. Un intercambio quita
     * dos aristas (a,b) y (c,d) y las reemplaza por (a,c) y (b,d), invirtiendo el tramo entre b y c.
     * Termina en un óptimo local: ningún intercambio mejora.
     * Complejidad: O(n^2) por pasada más O(n) por cada mejora aplicada; el número de pasadas
     * depende de la ruta inicial. Espacio O(n).
     * @return false si se detuvo por el límite de tiempo antes del óptimo local.
     */
    bool twoOpt(const std::vector<std::vector<double>>& dist, int depot, std::vector<int>& order,
                Clock::time_point deadline = noDeadline());

    struct IlsResult {
        std::vector<int> order;
        int iterations = 0;          // Iteraciones realizadas
        int improvements = 0;        // Iteraciones que mejoraron la ruta
        bool stoppedByTime = false;
    };

    /**
     * @brief Búsqueda local iterada: parte del 2-opt de la ruta inicial y en cada iteración la
     * perturba con un double-bridge aleatorio (corta la ruta en cuatro tramos A B C D y la arma
     * como A C B D), la mejora con 2-opt y se queda con ella si es más corta. La perturbación
     * saca a 2-opt de su óptimo local con un cambio que 2-opt no puede deshacer en un paso.
     *
     * Reproducible: la misma semilla y el mismo número de iteraciones dan la misma ruta, salvo que
     * el límite de tiempo la corte antes.
     * Complejidad: O(iteraciones · costo de 2-opt). Espacio O(n).
     */
    IlsResult iteratedLocalSearch(const std::vector<std::vector<double>>& dist, int depot,
                                  const std::vector<int>& initialOrder, uint64_t seed,
                                  int maxIterations = DEFAULT_ILS_ITERATIONS,
                                  Clock::time_point deadline = noDeadline());

} // namespace HeuristicSolver

} // namespace geodrone

#endif // GEODRONE_HEURISTICSOLVER_HPP
