#ifndef GEODRONE_EXACTSOLVER_HPP
#define GEODRONE_EXACTSOLVER_HPP

#include "../graph/VisibilityGraph.hpp"
#include <vector>
#include <string>
#include <chrono>

namespace geodrone {

/**
 * @brief Registro con el resultado de la optimización de la misión.
 */
struct MissionResult {
    bool feasible = false;
    double totalCost = 0.0;
    std::vector<int> tour;          // Secuencia ordenada de índices de nodos visitados
    std::vector<Point> fullPath;    // Trayectoria continua punto a punto en el plano R^2
    double executionTimeMs = 0.0;
    double peakMemoryMB = 0.0;
    std::string solverName = "";
    std::string message = "";
};

/**
 * @brief Parámetros operativos y restricciones para el solver.
 */
struct MissionConstraints {
    double maxBattery = 1000.0;     // Autonomía máxima normalizada Bmax
    double consumptionRate = 1.0;   // Factor de consumo alfa (unidades de batería / unidad de distancia)
    double timeLimitSeconds = 1.0;  // Tiempo límite competitivo (TL)
};

/**
 * @brief Solver exacto para instancias Small (1 <= N <= 10 clientes).
 * Implementa Branch & Bound con poda agresiva por cota inferior (MST / 1-Tree)
 * y alternativa DP con máscara de bits (Held-Karp).
 * Complejidad: O(2^N * N^2) en el peor caso con DP, o factorial podado O(N!).
 */
class ExactSolver {
private:
    MissionConstraints constraints_;

public:
    explicit ExactSolver(const MissionConstraints& constraints = MissionConstraints())
        : constraints_(constraints) {}

    /**
     * @brief Resuelve exactamente la misión de entrega minimizando la distancia euclidiana total.
     * @param costMatrix Matriz reducida de distancias |K| x |K| entre nodos clave.
     * @param keyNodeIndices Mapeo de índices clave hacia los índices del grafo de visibilidad.
     * @param clientCount Cantidad de clientes N a visitar obligatoriamente.
     * @param depotKeyIdx Índice del depósito en la matriz de distancias (típicamente 0).
     * @return MissionResult con el tour óptimo global o infactibilidad.
     */
    MissionResult solve(
        const std::vector<std::vector<double>>& costMatrix,
        const std::vector<int>& keyNodeIndices,
        int clientCount,
        int depotKeyIdx = 0);

    /**
     * @brief Actualiza los parámetros de misión.
     */
    void setConstraints(const MissionConstraints& constraints) {
        constraints_ = constraints;
    }

    const MissionConstraints& getConstraints() const {
        return constraints_;
    }

private:
    /**
     * @brief Calcula la cota inferior (Lower Bound) para el subconjunto de nodos no visitados mediante MST.
     */
    double computeMSTLowerBound(
        const std::vector<std::vector<double>>& costMatrix,
        const std::vector<int>& unvisited,
        int currentKeyNode,
        int depotKeyIdx);
};

} // namespace geodrone

#endif // GEODRONE_EXACTSOLVER_HPP
