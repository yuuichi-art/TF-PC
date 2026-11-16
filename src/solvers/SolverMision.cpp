#include "SolverMision.hpp"
#include "SolverExacto.hpp"
#include "SolverHeuristico.hpp"
#include "PlanificadorRecargas.hpp"

#include <algorithm>
#include <stdexcept>

namespace geodrone {

namespace {

using Clock = std::chrono::steady_clock;

double millisecondsSince(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// Fracción del tiempo límite que puede usar la búsqueda local iterada, con margen para
// insertar recargas, armar la trayectoria y medir.
const double TIME_BUDGET_FRACTION = 0.9;

} // namespace

std::string algorithmName(Algorithm algorithm) {
    switch (algorithm) {
        case Algorithm::BRANCH_AND_BOUND:       return "branch_and_bound";
        case Algorithm::NEAREST_NEIGHBOR:       return "nearest_neighbor";
        case Algorithm::MST_APPROXIMATION:      return "mst_approximation";
        case Algorithm::NEAREST_NEIGHBOR_2OPT:  return "nearest_neighbor_2opt";
        case Algorithm::MST_APPROXIMATION_2OPT: return "mst_approximation_2opt";
        case Algorithm::ITERATED_LOCAL_SEARCH:  return "iterated_local_search";
    }
    return "?";
}

const std::vector<Algorithm>& allAlgorithms() {
    static const std::vector<Algorithm> algorithms = {
        Algorithm::BRANCH_AND_BOUND, Algorithm::NEAREST_NEIGHBOR, Algorithm::MST_APPROXIMATION,
        Algorithm::NEAREST_NEIGHBOR_2OPT, Algorithm::MST_APPROXIMATION_2OPT, Algorithm::ITERATED_LOCAL_SEARCH
    };
    return algorithms;
}

MissionSolver::MissionSolver(const Escenario& escenario)
    : name_(escenario.nombre),
      seed_(escenario.semilla),
      timeLimitSeconds_(limitesDe(escenario.categoria).limiteTiempoSegundos),
      constructionStart_(Clock::now()),
      mision_(escenario),
      red_(mision_, escenario.bateria) {
    // Exclusiones (docs/decisiones.md §1): primero los que separan los obstáculos, luego los de batería.
    for (int c = 0; c < mision_.numClientes(); ++c) {
        int key = mision_.claveCliente(c);
        if (!mision_.conectadoABase(key)) {
            excluded_.push_back({mision_.etiqueta(key), "OBSTACLES"});
        } else if (!red_.puedeAtender(key)) {
            excluded_.push_back({mision_.etiqueta(key), "BATTERY"});
        } else {
            servable_.push_back(key);
        }
    }
    preparationMs_ = millisecondsSince(constructionStart_);
}

bool MissionSolver::supportsExact() const {
    return static_cast<int>(servable_.size()) <= ExactSolver::MAX_CLIENTS;
}

std::vector<int> MissionSolver::chooseOrder(Algorithm algorithm) const {
    const auto& dist = mision_.distancias();
    int base = mision_.claveBase();
    switch (algorithm) {
        case Algorithm::BRANCH_AND_BOUND:
            return ExactSolver::solve(dist, base, servable_).order;
        case Algorithm::NEAREST_NEIGHBOR:
            return HeuristicSolver::nearestNeighbor(dist, base, servable_);
        case Algorithm::MST_APPROXIMATION:
            return HeuristicSolver::mstApproximation(dist, base, servable_);
        case Algorithm::NEAREST_NEIGHBOR_2OPT: {
            std::vector<int> order = HeuristicSolver::nearestNeighbor(dist, base, servable_);
            HeuristicSolver::twoOpt(dist, base, order);
            return order;
        }
        case Algorithm::MST_APPROXIMATION_2OPT: {
            std::vector<int> order = HeuristicSolver::mstApproximation(dist, base, servable_);
            HeuristicSolver::twoOpt(dist, base, order);
            return order;
        }
        case Algorithm::ITERATED_LOCAL_SEARCH: {
            // Presupuesto: el 90 % del límite de la categoría menos lo que ya costó la preparación.
            double budgetMs = std::max(0.0, TIME_BUDGET_FRACTION * timeLimitSeconds_ * 1000.0 - preparationMs_);
            auto deadline = Clock::now() + std::chrono::microseconds(static_cast<long long>(budgetMs * 1000.0));
            std::vector<int> start = HeuristicSolver::nearestNeighbor(dist, base, servable_);
            return HeuristicSolver::iteratedLocalSearch(dist, base, start, seed_,
                                                        HeuristicSolver::DEFAULT_ILS_ITERATIONS, deadline).order;
        }
    }
    return {};
}

// Cada tramo entre nodos clave se reemplaza por su camino real en el grafo de visibilidad.
std::vector<RoutePoint> MissionSolver::expandPath(const std::vector<int>& keySequence) const {
    const GrafoVisibilidad& grafo = mision_.grafo();
    auto toPoint = [&](int node) {
        const Nodo& n = grafo.nodo(node);
        return RoutePoint{n.punto, n.esNodoClave() ? n.nombre : "V"};
    };

    std::vector<RoutePoint> path;
    if (keySequence.empty()) return path;
    path.push_back(toPoint(keySequence.front()));
    for (size_t i = 0; i + 1 < keySequence.size(); ++i) {
        std::vector<int> leg = mision_.nodosCamino(keySequence[i], keySequence[i + 1]);
        for (size_t j = 1; j < leg.size(); ++j) {   // El primer nodo ya está en el camino
            path.push_back(toPoint(leg[j]));
        }
    }
    return path;
}

Route MissionSolver::solve(Algorithm algorithm) const {
    if (algorithm == Algorithm::BRANCH_AND_BOUND && !supportsExact()) {
        throw std::invalid_argument("el solver exacto admite hasta " + std::to_string(ExactSolver::MAX_CLIENTS) +
                                    " clientes y este escenario tiene " + std::to_string(servable_.size()));
    }
    auto start = Clock::now();

    Route route;
    route.escenario = name_;
    route.algorithm = algorithmName(algorithm);
    route.excluded = excluded_;
    route.preparationMs = preparationMs_;

    RechargePlan plan = RechargePlanner(mision_, red_).plan(chooseOrder(algorithm));
    route.feasible = plan.feasible;
    route.costo = plan.costo;
    route.recharges = plan.recharges;
    for (int key : plan.keySequence) {
        route.order.push_back(mision_.etiqueta(key));
    }
    route.path = expandPath(plan.keySequence);

    route.solveMs = millisecondsSince(start);
    return route;
}

} // namespace geodrone
