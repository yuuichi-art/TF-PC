#ifndef GEODRONE_MISSIONSOLVER_HPP
#define GEODRONE_MISSIONSOLVER_HPP

#include "Route.hpp"
#include "../graph/ChargingNetwork.hpp"
#include "../graph/MissionGraph.hpp"
#include "../scenario/Scenario.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Algoritmos que deciden el orden de visita. El nombre es el que aparece en los
 * archivos de ruta y en el CSV del juez.
 */
enum class Algorithm {
    BRANCH_AND_BOUND,           // Exacto, solo Small (hasta ExactSolver::MAX_CLIENTS clientes)
    NEAREST_NEIGHBOR,
    MST_APPROXIMATION,
    NEAREST_NEIGHBOR_2OPT,
    MST_APPROXIMATION_2OPT,
    ITERATED_LOCAL_SEARCH       // Parte del vecino más cercano
};

std::string algorithmName(Algorithm algorithm);
const std::vector<Algorithm>& allAlgorithms();

/**
 * @brief Resuelve un escenario completo:
 * 1. Prepara una sola vez el grafo de misión y la red de recarga.
 * 2. Excluye los clientes que los obstáculos separan de la base (DFS) y los que la batería no
 *    permite atender (2·dRecarga > B).
 * 3. Para cada algoritmo pedido: orden de visita, inserción de recargas y trayectoria real.
 *
 * Preparar una vez y resolver varias permite que el juez compare algoritmos sin reconstruir
 * el grafo. El tiempo de una ruta es preparación + resolución, como si se resolviera sola.
 *
 * Guarda el grafo y la red como miembros, que se referencian entre sí: no se puede copiar.
 */
class MissionSolver {
public:
    explicit MissionSolver(const Scenario& scenario);
    MissionSolver(const MissionSolver&) = delete;
    MissionSolver& operator=(const MissionSolver&) = delete;

    // El exacto solo se puede usar con pocos clientes atendibles.
    bool supportsExact() const;

    /**
     * @brief Resuelve con el algoritmo dado. La búsqueda local iterada se detiene antes de llegar
     * al 90 % del tiempo límite de la categoría, descontando la preparación.
     * Lanza std::invalid_argument si se pide el exacto y supportsExact() es false.
     */
    Route solve(Algorithm algorithm) const;

    const MissionGraph& mission() const { return mission_; }
    const ChargingNetwork& network() const { return network_; }
    const std::vector<ExcludedClient>& excluded() const { return excluded_; }
    const std::vector<int>& servableClients() const { return servable_; }   // Claves
    double preparationMs() const { return preparationMs_; }

private:
    std::string name_;
    uint64_t seed_;
    double timeLimitSeconds_;
    double preparationMs_ = 0.0;
    // Los miembros se construyen en el orden en que se declaran: este va antes que el grafo
    // para que la medición de la preparación lo incluya.
    std::chrono::steady_clock::time_point constructionStart_;
    MissionGraph mission_;
    ChargingNetwork network_;              // Referencia a mission_: se declara después
    std::vector<int> servable_;
    std::vector<ExcludedClient> excluded_;

    std::vector<int> chooseOrder(Algorithm algorithm) const;
    std::vector<RoutePoint> expandPath(const std::vector<int>& keySequence) const;
};

} // namespace geodrone

#endif // GEODRONE_MISSIONSOLVER_HPP
