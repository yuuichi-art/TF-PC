#ifndef GEODRONE_EXACTSOLVER_HPP
#define GEODRONE_EXACTSOLVER_HPP

#include <vector>

namespace geodrone {

/**
 * @brief Resultado del solver exacto.
 */
struct ExactResult {
    std::vector<int> order;         // Claves de los clientes en el orden de visita óptimo
    double cost = 0.0;              // Costo del ciclo base -> clientes -> base, sin recargas
    long long nodesExplored = 0;    // Nodos del árbol de búsqueda visitados
    long long nodesPruned = 0;      // Ramas descartadas por la cota
};

/**
 * @brief Solver exacto para instancias Small: branch and bound sobre el orden de visita (RF07).
 *
 * Recorre en profundidad el árbol de órdenes posibles: cada nivel elige el siguiente cliente.
 * En cada nodo calcula una cota inferior del costo que falta y descarta la rama si
 * costo acumulado + cota ≥ mejor ruta conocida.
 *
 * Cota inferior: árbol de expansión mínima (Prim) de {posición actual, clientes pendientes,
 * base}. Lo que falta recorrer es un camino que une exactamente esos nodos, y todo camino es
 * un árbol, así que nunca cuesta menos que el árbol mínimo.
 *
 * Para encontrar pronto una buena ruta, y así podar más, los hijos se exploran del cliente
 * más cercano al más lejano.
 *
 * Optimiza el orden sobre la matriz de distancias (docs/decisiones.md §3): las recargas se insertan
 * después, así que el óptimo es el del orden, no el de la ruta con recargas.
 *
 * Complejidad: O(n! · n^2) en el peor caso (n! órdenes, Prim O(n^2) por nodo); la poda reduce
 * muchísimo los nodos visitados en la práctica (ver nodesExplored). Espacio O(n) de recursión.
 * Límite práctico: MAX_CLIENTS.
 */
class ExactSolver {
public:
    static constexpr int MAX_CLIENTS = 12;

    /**
     * @param dist     Matriz de distancias entre nodos clave (la del grafo de misión).
     * @param depot    Clave de la base.
     * @param clients  Claves de los clientes a visitar. Deben ser alcanzables desde la base.
     * Lanza std::invalid_argument si hay más de MAX_CLIENTS clientes.
     */
    static ExactResult solve(const std::vector<std::vector<double>>& dist, int depot,
                             const std::vector<int>& clients);
};

} // namespace geodrone

#endif // GEODRONE_EXACTSOLVER_HPP
