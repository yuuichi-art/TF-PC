#ifndef GEODRONE_HEURISTICSOLVER_HPP
#define GEODRONE_HEURISTICSOLVER_HPP

#include "ExactSolver.hpp" // Para MissionResult y MissionConstraints
#include "../graph/VisibilityGraph.hpp"
#include <vector>
#include <string>

namespace geodrone {

/**
 * @brief Solver heurístico para instancias Medium, Large y Extreme (11 <= N <= 1000).
 * Integra una fase constructiva Nearest Neighbor (Vecino Más Cercano) con gestión
 * de recargas de batería, seguida de optimización local iterativa 2-Opt Swap.
 * Complejidad: O(N^2) fase constructiva + O(k * N^2) en búsqueda local 2-Opt.
 */
class HeuristicSolver {
private:
    MissionConstraints constraints_;
    int max2OptIterations_ = 1000;

public:
    explicit HeuristicSolver(
        const MissionConstraints& constraints = MissionConstraints(),
        int max2OptIterations = 1000)
        : constraints_(constraints), max2OptIterations_(max2OptIterations) {}

    /**
     * @brief Ejecuta el pipeline heurístico: Nearest Neighbor + 2-Opt.
     * @param costMatrix Matriz reducida de distancias |K| x |K| entre nodos clave.
     * @param keyNodeIndices Mapeo de índices clave hacia los índices del grafo de visibilidad.
     * @param keyNodeTypes Tipos de nodo de cada elemento en la matriz reducida.
     * @param depotKeyIdx Índice del depósito (típicamente 0).
     * @return MissionResult con el tour factible de alta calidad y métricas de CPU.
     */
    MissionResult solve(
        const std::vector<std::vector<double>>& costMatrix,
        const std::vector<int>& keyNodeIndices,
        const std::vector<NodeType>& keyNodeTypes,
        int depotKeyIdx = 0);

    /**
     * @brief Fase constructiva: Nearest Neighbor considerando autonomía de batería y estaciones.
     */
    std::vector<int> constructNearestNeighbor(
        const std::vector<std::vector<double>>& costMatrix,
        const std::vector<NodeType>& keyNodeTypes,
        int depotKeyIdx);

    /**
     * @brief Fase de mejora local: 2-Opt swap para invertir aristas y eliminar cruces espaciales.
     */
    void apply2OptOptimization(
        std::vector<int>& tour,
        const std::vector<std::vector<double>>& costMatrix);

    void setConstraints(const MissionConstraints& constraints) {
        constraints_ = constraints;
    }

    void setMax2OptIterations(int iters) {
        max2OptIterations_ = iters;
    }
};

} // namespace geodrone

#endif // GEODRONE_HEURISTICSOLVER_HPP
