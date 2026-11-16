// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/solvers/ExactSolver.hpp"
#include "../src/solvers/HeuristicSolver.hpp"
#include "../src/graph/MissionGraph.hpp"
#include "../src/graph/Mst.hpp"
#include "../src/scenario/DatasetGenerator.hpp"
#include "../src/scenario/ScenarioIO.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;
using Matrix = std::vector<std::vector<double>>;

const double TOLERANCE = 1e-6;

// Costo del ciclo base -> clientes en ese orden -> base.
double tourCost(const Matrix& dist, int depot, const std::vector<int>& order) {
    double cost = 0.0;
    int position = depot;
    for (int client : order) {
        cost += dist[position][client];
        position = client;
    }
    return cost + dist[position][depot];
}

// Fuerza bruta: prueba todos los órdenes. O(n! · n); solo para n pequeño.
double bruteForce(const Matrix& dist, int depot, std::vector<int> clients) {
    std::sort(clients.begin(), clients.end());
    double best = INF;
    do {
        best = std::min(best, tourCost(dist, depot, clients));
    } while (std::next_permutation(clients.begin(), clients.end()));
    return best;
}

bool isPermutationOf(std::vector<int> order, std::vector<int> clients) {
    std::sort(order.begin(), order.end());
    std::sort(clients.begin(), clients.end());
    return order == clients;
}

Matrix euclideanMatrix(const std::vector<Point>& points) {
    Matrix dist(points.size(), std::vector<double>(points.size()));
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = 0; j < points.size(); ++j) dist[i][j] = points[i].dist(points[j]);
    }
    return dist;
}

// El exacto devuelve una permutación de los clientes, su costo es el informado y coincide
// con el de la fuerza bruta.
ExactResult assertOptimal(const Matrix& dist, int depot, const std::vector<int>& clients) {
    ExactResult result = ExactSolver::solve(dist, depot, clients);
    assert(isPermutationOf(result.order, clients));
    assert(std::abs(tourCost(dist, depot, result.order) - result.cost) < TOLERANCE);
    assert(std::abs(result.cost - bruteForce(dist, depot, clients)) < TOLERANCE);
    return result;
}

void testHandExamples() {
    std::cout << "[TEST] 1. Casos calculados a mano..." << std::endl;

    // Cuadrado de lado 10 con la base en una esquina: el óptimo es el perímetro, 40.
    Matrix square = euclideanMatrix({Point(0, 0), Point(10, 0), Point(10, 10), Point(0, 10)});
    assert(std::abs(assertOptimal(square, 0, {1, 2, 3}).cost - 40.0) < TOLERANCE);

    // Puntos en una recta: base en 0 y clientes en 1, -2 y 4. El vecino más cercano haría
    // 0 -> 1 -> -2 -> 4 -> 0 = 14; el óptimo es 12 (por ejemplo 0 -> 1 -> 4 -> -2 -> 0).
    Matrix line = euclideanMatrix({Point(0, 0), Point(1, 0), Point(-2, 0), Point(4, 0)});
    assert(std::abs(tourCost(line, 0, {1, 2, 3}) - 14.0) < TOLERANCE);
    assert(std::abs(assertOptimal(line, 0, {1, 2, 3}).cost - 12.0) < TOLERANCE);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testEdgeCases() {
    std::cout << "[TEST] 2. Casos borde: sin clientes, un cliente y demasiados clientes..." << std::endl;

    Matrix dist = euclideanMatrix({Point(0, 0), Point(3, 4)});
    ExactResult empty = ExactSolver::solve(dist, 0, {});
    assert(empty.order.empty() && empty.cost == 0.0);

    ExactResult single = ExactSolver::solve(dist, 0, {1});
    assert(single.order == std::vector<int>{1} && std::abs(single.cost - 10.0) < TOLERANCE);

    std::vector<int> tooMany(ExactSolver::MAX_CLIENTS + 1, 1);
    bool rejected = false;
    try {
        ExactSolver::solve(dist, 0, tooMany);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testRandomAgainstBruteForce() {
    std::cout << "[TEST] 3. Instancias aleatorias de 1 a 8 clientes contra fuerza bruta..." << std::endl;

    std::mt19937 rng(2026);
    std::uniform_int_distribution<int> coordinate(0, 100);
    for (int n = 1; n <= 8; ++n) {
        for (int instance = 0; instance < 10; ++instance) {
            std::vector<Point> points;
            for (int i = 0; i <= n; ++i) points.emplace_back(coordinate(rng), coordinate(rng));
            std::vector<int> clients;
            for (int i = 1; i <= n; ++i) clients.push_back(i);
            assertOptimal(euclideanMatrix(points), 0, clients);
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testScenariosWithObstacles(const std::string& adversarialFolder) {
    std::cout << "[TEST] 4. Escenarios con obstaculos (generados y adversariales) contra fuerza bruta..." << std::endl;

    std::vector<Scenario> scenarios;
    for (uint64_t seed = 1; seed <= 15; ++seed) {
        scenarios.push_back(DatasetGenerator::generateLayout(
            DatasetGenerator::specFor(DatasetCategory::SMALL), DatasetCategory::SMALL, seed));
    }
    for (const char* name : {"adv_collinear", "adv_enclosed", "adv_corridor", "adv_concave_trap"}) {
        scenarios.push_back(ScenarioIO::load(adversarialFolder + "/" + name + ".txt"));
    }

    for (const Scenario& s : scenarios) {
        MissionGraph mission(s);
        std::vector<int> unreachable = mission.unreachableClients();
        std::vector<int> clients;
        for (int c = 0; c < mission.numClients() && clients.size() < 8; ++c) {
            if (std::find(unreachable.begin(), unreachable.end(), c) == unreachable.end()) {
                clients.push_back(mission.clientKey(c));
            }
        }
        assertOptimal(mission.distances(), mission.depotKey(), clients);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testPruning() {
    std::cout << "[TEST] 5. La poda evita recorrer todos los ordenes..." << std::endl;

    // Con 10 clientes hay 10! = 3 628 800 órdenes completos; el árbol entero tiene aún más nodos.
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> coordinate(0, 1000);
    for (int instance = 0; instance < 5; ++instance) {
        std::vector<Point> points;
        for (int i = 0; i <= 10; ++i) points.emplace_back(coordinate(rng), coordinate(rng));
        std::vector<int> clients;
        for (int i = 1; i <= 10; ++i) clients.push_back(i);
        ExactResult result = ExactSolver::solve(euclideanMatrix(points), 0, clients);
        std::cout << "  10 clientes: " << result.nodesExplored << " nodos visitados, "
                  << result.nodesPruned << " podas" << std::endl;
        assert(result.nodesExplored < 3628800 / 10);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

// ---------------------------------------------------------------------------------------------
// Heurísticas
// ---------------------------------------------------------------------------------------------

struct Instance {
    Matrix dist;
    int depot;
    std::vector<int> clients;
};

Instance randomInstance(int n, std::mt19937& rng) {
    std::uniform_int_distribution<int> coordinate(0, 1000);
    std::vector<Point> points;
    for (int i = 0; i <= n; ++i) points.emplace_back(coordinate(rng), coordinate(rng));
    Instance instance{euclideanMatrix(points), 0, {}};
    for (int i = 1; i <= n; ++i) instance.clients.push_back(i);
    return instance;
}

// Instancia con obstáculos a partir de un escenario generado (solo geometría).
Instance scenarioInstance(DatasetCategory category, uint64_t seed) {
    MissionGraph mission(DatasetGenerator::generateLayout(DatasetGenerator::specFor(category), category, seed));
    Instance instance{mission.distances(), mission.depotKey(), {}};
    for (int c = 0; c < mission.numClients(); ++c) instance.clients.push_back(mission.clientKey(c));
    return instance;
}

// Ningún intercambio 2-opt acorta la ruta: es un óptimo local.
bool isTwoOptLocalOptimum(const Matrix& dist, int depot, const std::vector<int>& order) {
    std::vector<int> tour = {depot};
    tour.insert(tour.end(), order.begin(), order.end());
    tour.push_back(depot);
    int n = static_cast<int>(order.size());
    for (int i = 1; i < n; ++i) {
        for (int j = i + 1; j <= n; ++j) {
            double delta = dist[tour[i - 1]][tour[j]] + dist[tour[i]][tour[j + 1]]
                         - dist[tour[i - 1]][tour[i]] - dist[tour[j]][tour[j + 1]];
            if (delta < -1e-6) return false;
        }
    }
    return true;
}

void testHeuristicHandExamples() {
    std::cout << "[TEST] 6. Heuristicas en los casos calculados a mano..." << std::endl;

    // Recta: base en 0, clientes en 1, -2 y 4. El vecino más cercano hace 0 -> 1 -> -2 -> 4 -> 0 = 14
    // y 2-opt lo corrige hasta el óptimo, 12.
    Matrix line = euclideanMatrix({Point(0, 0), Point(1, 0), Point(-2, 0), Point(4, 0)});
    std::vector<int> greedy = HeuristicSolver::nearestNeighbor(line, 0, {1, 2, 3});
    assert(greedy == std::vector<int>({1, 2, 3}));
    assert(std::abs(HeuristicSolver::tourCost(line, 0, greedy) - 14.0) < TOLERANCE);
    assert(HeuristicSolver::twoOpt(line, 0, greedy));
    assert(std::abs(HeuristicSolver::tourCost(line, 0, greedy) - 12.0) < TOLERANCE);

    // Sin clientes, todas devuelven una ruta vacía.
    assert(HeuristicSolver::nearestNeighbor(line, 0, {}).empty());
    assert(HeuristicSolver::mstApproximation(line, 0, {}).empty());
    assert(HeuristicSolver::iteratedLocalSearch(line, 0, {}, 1).order.empty());

    std::cout << "  -> PASSED\n" << std::endl;
}

void testHeuristicsProduceValidOrders() {
    std::cout << "[TEST] 7. Cada heuristica devuelve una permutacion; 2-opt nunca empeora..." << std::endl;

    std::mt19937 rng(42);
    std::vector<Instance> instances;
    for (int n : {1, 2, 3, 4, 5, 20, 60}) instances.push_back(randomInstance(n, rng));
    instances.push_back(scenarioInstance(DatasetCategory::MEDIUM, 1));
    instances.push_back(scenarioInstance(DatasetCategory::LARGE, 1));

    for (const Instance& in : instances) {
        std::vector<int> greedy = HeuristicSolver::nearestNeighbor(in.dist, in.depot, in.clients);
        std::vector<int> approx = HeuristicSolver::mstApproximation(in.dist, in.depot, in.clients);
        assert(isPermutationOf(greedy, in.clients) && isPermutationOf(approx, in.clients));

        for (std::vector<int> start : {greedy, approx}) {
            double before = HeuristicSolver::tourCost(in.dist, in.depot, start);
            assert(HeuristicSolver::twoOpt(in.dist, in.depot, start));
            assert(isPermutationOf(start, in.clients));
            assert(HeuristicSolver::tourCost(in.dist, in.depot, start) <= before + TOLERANCE);
            assert(isTwoOptLocalOptimum(in.dist, in.depot, start));
        }

        HeuristicSolver::IlsResult ils = HeuristicSolver::iteratedLocalSearch(in.dist, in.depot, greedy, 7, 30);
        assert(isPermutationOf(ils.order, in.clients) && !ils.stoppedByTime);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testApproximationGuarantee() {
    std::cout << "[TEST] 8. La 2-aproximacion cumple costo <= 2 x arbol minimo <= 2 x optimo..." << std::endl;

    std::mt19937 rng(9);
    for (int n = 1; n <= 9; ++n) {
        for (int instance = 0; instance < 5; ++instance) {
            Instance in = randomInstance(n, rng);
            double approx = HeuristicSolver::tourCost(in.dist, in.depot,
                                                      HeuristicSolver::mstApproximation(in.dist, in.depot, in.clients));
            double optimum = ExactSolver::solve(in.dist, in.depot, in.clients).cost;
            assert(approx <= 2.0 * optimum + TOLERANCE);
        }
    }

    // En instancias grandes no hay óptimo para comparar, pero sí el árbol mínimo, que es menor.
    for (uint64_t seed = 1; seed <= 3; ++seed) {
        Instance in = scenarioInstance(DatasetCategory::LARGE, seed);
        std::vector<int> nodes = {in.depot};
        nodes.insert(nodes.end(), in.clients.begin(), in.clients.end());
        double treeCost = primMst(in.dist, nodes).cost;
        double approx = HeuristicSolver::tourCost(in.dist, in.depot,
                                                  HeuristicSolver::mstApproximation(in.dist, in.depot, in.clients));
        assert(treeCost <= approx + TOLERANCE && approx <= 2.0 * treeCost + TOLERANCE);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testIteratedLocalSearch() {
    std::cout << "[TEST] 9. Busqueda local iterada: mejora, es reproducible y respeta el tiempo..." << std::endl;

    // Nunca peor que el 2-opt de su ruta inicial, y la misma semilla da la misma ruta.
    for (uint64_t seed = 1; seed <= 3; ++seed) {
        Instance in = scenarioInstance(DatasetCategory::MEDIUM, seed);
        std::vector<int> start = HeuristicSolver::nearestNeighbor(in.dist, in.depot, in.clients);
        std::vector<int> localOptimum = start;
        HeuristicSolver::twoOpt(in.dist, in.depot, localOptimum);

        HeuristicSolver::IlsResult first = HeuristicSolver::iteratedLocalSearch(in.dist, in.depot, start, seed);
        HeuristicSolver::IlsResult second = HeuristicSolver::iteratedLocalSearch(in.dist, in.depot, start, seed);
        assert(first.order == second.order);
        assert(first.iterations == HeuristicSolver::DEFAULT_ILS_ITERATIONS);
        assert(HeuristicSolver::tourCost(in.dist, in.depot, first.order) <=
               HeuristicSolver::tourCost(in.dist, in.depot, localOptimum) + TOLERANCE);
    }

    // En instancias Small llega al óptimo del branch and bound.
    for (uint64_t seed = 1; seed <= 10; ++seed) {
        Instance in = scenarioInstance(DatasetCategory::SMALL, seed);
        std::vector<int> start = HeuristicSolver::nearestNeighbor(in.dist, in.depot, in.clients);
        double ils = HeuristicSolver::tourCost(in.dist, in.depot,
                                               HeuristicSolver::iteratedLocalSearch(in.dist, in.depot, start, seed).order);
        double optimum = ExactSolver::solve(in.dist, in.depot, in.clients).cost;
        assert(ils <= optimum + TOLERANCE);
    }

    // Con un límite de 30 ms y muchas iteraciones pedidas, se detiene a tiempo.
    Instance large = scenarioInstance(DatasetCategory::LARGE, 2);
    std::vector<int> start = HeuristicSolver::nearestNeighbor(large.dist, large.depot, large.clients);
    auto begin = HeuristicSolver::Clock::now();
    HeuristicSolver::IlsResult limited = HeuristicSolver::iteratedLocalSearch(
        large.dist, large.depot, start, 1, 1000000, begin + std::chrono::milliseconds(30));
    double elapsedMs = std::chrono::duration<double, std::milli>(HeuristicSolver::Clock::now() - begin).count();
    assert(limited.stoppedByTime && isPermutationOf(limited.order, large.clients));
    assert(elapsedMs < 30.0 + 200.0);   // Margen para la última pasada de 2-opt y el sistema

    std::cout << "  -> PASSED\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string adversarialFolder = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DE LOS SOLVERS           " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    testHandExamples();
    testEdgeCases();
    testRandomAgainstBruteForce();
    testScenariosWithObstacles(adversarialFolder);
    testPruning();
    testHeuristicHandExamples();
    testHeuristicsProduceValidOrders();
    testApproximationGuarantee();
    testIteratedLocalSearch();

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DE LOS SOLVERS PASARON              " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
