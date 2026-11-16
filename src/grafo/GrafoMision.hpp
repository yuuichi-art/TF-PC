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
class GrafoMision {
public:
    explicit GrafoMision(const Escenario& escenario);

    int numNodosClave() const { return 1 + numEstaciones_ + numClientes_; }
    int numEstaciones() const { return numEstaciones_; }
    int numClientes() const { return numClientes_; }

    int claveBase() const { return 0; }
    int claveEstacion(int estacion) const { return 1 + estacion; }
    int claveCliente(int cliente) const { return 1 + numEstaciones_ + cliente; }

    const Punto& puntoClave(int clave) const { return grafo_.nodo(clave).punto; }

    // Etiqueta del nodo clave: "D0", "S1".."S<S>" o "C1".."C<N>", como en los archivos de ruta.
    const std::string& etiqueta(int clave) const { return grafo_.nodo(clave).nombre; }

    // Distancia de vuelo mínima entre dos nodos clave (infinito si no hay camino).
    double distancia(int claveOrigen, int claveDestino) const { return distancias_[claveOrigen][claveDestino]; }
    const std::vector<std::vector<double>>& distancias() const { return distancias_; }

    // Resultado del DFS desde la base.
    bool conectadoABase(int clave) const { return conectados_[clave] != 0; }

    // Clientes (índices 0..N-1) que los obstáculos separan de la base.
    std::vector<int> clientesInalcanzables() const;

    // Nodos del grafo de visibilidad que forma el camino mínimo entre dos nodos clave,
    // incluidos ambos extremos. Vacío si no hay camino.
    std::vector<int> nodosCamino(int claveOrigen, int claveDestino) const;

    const GrafoVisibilidad& grafo() const { return grafo_; }

private:
    GrafoVisibilidad grafo_;
    int numEstaciones_ = 0;
    int numClientes_ = 0;
    std::vector<std::vector<double>> distancias_;     // distancias_[a][b] entre nodos clave
    std::vector<std::vector<int>> padres_;      // padres_[a][v]: predecesor de v en el camino desde a
    std::vector<char> conectados_;               // conectados_[v]: v se alcanza desde la base

    void marcarConectados(int nodo);
};

} // namespace geodrone

#endif // GEODRONE_GRAFOMISION_HPP
