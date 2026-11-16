// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/solvers/SolverExacto.hpp"
#include "../src/solvers/SolverHeuristico.hpp"
#include "../src/grafo/GrafoMision.hpp"
#include "../src/grafo/ArbolMinimo.hpp"
#include "../src/escenario/GeneradorDatasets.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"

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
double tourCost(const Matrix& dist, int base, const std::vector<int>& order) {
    double cost = 0.0;
    int position = base;
    for (int client : order) {
        cost += dist[position][client];
        position = client;
    }
    return cost + dist[position][base];
}

// Fuerza bruta: prueba todos los órdenes. O(n! · n); solo para n pequeño.
double bruteForce(const Matrix& dist, int base, std::vector<int> clientes) {
    std::sort(clientes.begin(), clientes.end());
    double best = INF;
    do {
        best = std::min(best, tourCost(dist, base, clientes));
    } while (std::next_permutation(clientes.begin(), clientes.end()));
    return best;
}

bool isPermutationOf(std::vector<int> order, std::vector<int> clientes) {
    std::sort(order.begin(), order.end());
    std::sort(clientes.begin(), clientes.end());
    return order == clientes;
}

Matrix euclideanMatrix(const std::vector<Punto>& points) {
    Matrix dist(points.size(), std::vector<double>(points.size()));
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = 0; j < points.size(); ++j) dist[i][j] = points[i].distancia(points[j]);
    }
    return dist;
}

// El exacto devuelve una permutación de los clientes, su costo es el informado y coincide
// con el de la fuerza bruta.
ExactResult assertOptimal(const Matrix& dist, int base, const std::vector<int>& clientes) {
    ExactResult result = ExactSolver::solve(dist, base, clientes);
    assert(isPermutationOf(result.order, clientes));
    assert(std::abs(tourCost(dist, base, result.order) - result.cost) < TOLERANCE);
    assert(std::abs(result.cost - bruteForce(dist, base, clientes)) < TOLERANCE);
    return result;
}

void testHandExamples() {
    std::cout << "[TEST] 1. Casos calculados a mano..." << std::endl;

    // Cuadrado de lado 10 con la base en una esquina: el óptimo es el perímetro, 40.
    Matrix square = euclideanMatrix({Punto(0, 0), Punto(10, 0), Punto(10, 10), Punto(0, 10)});
    assert(std::abs(assertOptimal(square, 0, {1, 2, 3}).cost - 40.0) < TOLERANCE);

    // Puntos en una recta: base en 0 y clientes en 1, -2 y 4. El vecino más cercano haría
    // 0 -> 1 -> -2 -> 4 -> 0 = 14; el óptimo es 12 (por ejemplo 0 -> 1 -> 4 -> -2 -> 0).
    Matrix line = euclideanMatrix({Punto(0, 0), Punto(1, 0), Punto(-2, 0), Punto(4, 0)});
    assert(std::abs(tourCost(line, 0, {1, 2, 3}) - 14.0) < TOLERANCE);
    assert(std::abs(assertOptimal(line, 0, {1, 2, 3}).cost - 12.0) < TOLERANCE);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testEdgeCases() {
    std::cout << "[TEST] 2. Casos borde: sin clientes, un cliente y demasiados clientes..." << std::endl;

    Matrix dist = euclideanMatrix({Punto(0, 0), Punto(3, 4)});
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
            std::vector<Punto> points;
            for (int i = 0; i <= n; ++i) points.emplace_back(coordinate(rng), coordinate(rng));
            std::vector<int> clientes;
            for (int i = 1; i <= n; ++i) clientes.push_back(i);
            assertOptimal(euclideanMatrix(points), 0, clientes);
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testScenariosWithObstacles(const std::string& adversarialFolder) {
    std::cout << "[TEST] 4. Escenarios con obstaculos (generados y adversariales) contra fuerza bruta..." << std::endl;

    std::vector<Escenario> scenarios;
    for (uint64_t semilla = 1; semilla <= 15; ++semilla) {
        scenarios.push_back(GeneradorDatasets::generarGeometria(
            GeneradorDatasets::rangosDe(Categoria::SMALL), Categoria::SMALL, semilla));
    }
    for (const char* nombre : {"adv_collinear", "adv_enclosed", "adv_corridor", "adv_concave_trap"}) {
        scenarios.push_back(ArchivoEscenario::cargar(adversarialFolder + "/" + nombre + ".txt"));
    }

    for (const Escenario& s : scenarios) {
        MissionGraph mission(s);
        std::vector<int> unreachable = mission.unreachableClients();
        std::vector<int> clientes;
        for (int c = 0; c < mission.numClientes() && clientes.size() < 8; ++c) {
            if (std::find(unreachable.begin(), unreachable.end(), c) == unreachable.end()) {
                clientes.push_back(mission.clientKey(c));
            }
        }
        assertOptimal(mission.distances(), mission.depotKey(), clientes);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testPruning() {
    std::cout << "[TEST] 5. La poda evita recorrer todos los ordenes..." << std::endl;

    // Con 10 clientes hay 10! = 3 628 800 órdenes completos; el árbol entero tiene aún más nodos.
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> coordinate(0, 1000);
    for (int instance = 0; instance < 5; ++instance) {
        std::vector<Punto> points;
        for (int i = 0; i <= 10; ++i) points.emplace_back(coordinate(rng), coordinate(rng));
        std::vector<int> clientes;
        for (int i = 1; i <= 10; ++i) clientes.push_back(i);
        ExactResult result = ExactSolver::solve(euclideanMatrix(points), 0, clientes);
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
    int base;
    std::vector<int> clientes;
};

Instance randomInstance(int n, std::mt19937& rng) {
    std::uniform_int_distribution<int> coordinate(0, 1000);
    std::vector<Punto> points;
    for (int i = 0; i <= n; ++i) points.emplace_back(coordinate(rng), coordinate(rng));
    Instance instance{euclideanMatrix(points), 0, {}};
    for (int i = 1; i <= n; ++i) instance.clientes.push_back(i);
    return instance;
}

// Instancia con obstáculos a partir de un escenario generado (solo geometría).
Instance scenarioInstance(Categoria categoria, uint64_t semilla) {
    MissionGraph mission(GeneradorDatasets::generarGeometria(GeneradorDatasets::rangosDe(categoria), categoria, semilla));
    Instance instance{mission.distances(), mission.depotKey(), {}};
    for (int c = 0; c < mission.numClientes(); ++c) instance.clientes.push_back(mission.clientKey(c));
    return instance;
}

// Ningún intercambio 2-opt acorta la ruta: es un óptimo local.
bool isTwoOptLocalOptimum(const Matrix& dist, int base, const std::vector<int>& order) {
    std::vector<int> tour = {base};
    tour.insert(tour.end(), order.begin(), order.end());
    tour.push_back(base);
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
    Matrix line = euclideanMatrix({Punto(0, 0), Punto(1, 0), Punto(-2, 0), Punto(4, 0)});
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
    instances.push_back(scenarioInstance(Categoria::MEDIUM, 1));
    instances.push_back(scenarioInstance(Categoria::LARGE, 1));

    for (const Instance& in : instances) {
        std::vector<int> greedy = HeuristicSolver::nearestNeighbor(in.dist, in.base, in.clientes);
        std::vector<int> approx = HeuristicSolver::mstApproximation(in.dist, in.base, in.clientes);
        assert(isPermutationOf(greedy, in.clientes) && isPermutationOf(approx, in.clientes));

        for (std::vector<int> start : {greedy, approx}) {
            double before = HeuristicSolver::tourCost(in.dist, in.base, start);
            assert(HeuristicSolver::twoOpt(in.dist, in.base, start));
            assert(isPermutationOf(start, in.clientes));
            assert(HeuristicSolver::tourCost(in.dist, in.base, start) <= before + TOLERANCE);
            assert(isTwoOptLocalOptimum(in.dist, in.base, start));
        }

        HeuristicSolver::IlsResult ils = HeuristicSolver::iteratedLocalSearch(in.dist, in.base, greedy, 7, 30);
        assert(isPermutationOf(ils.order, in.clientes) && !ils.stoppedByTime);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testApproximationGuarantee() {
    std::cout << "[TEST] 8. La 2-aproximacion cumple costo <= 2 x arbol minimo <= 2 x optimo..." << std::endl;

    std::mt19937 rng(9);
    for (int n = 1; n <= 9; ++n) {
        for (int instance = 0; instance < 5; ++instance) {
            Instance in = randomInstance(n, rng);
            double approx = HeuristicSolver::tourCost(in.dist, in.base,
                                                      HeuristicSolver::mstApproximation(in.dist, in.base, in.clientes));
            double optimum = ExactSolver::solve(in.dist, in.base, in.clientes).cost;
            assert(approx <= 2.0 * optimum + TOLERANCE);
        }
    }

    // En instancias grandes no hay óptimo para comparar, pero sí el árbol mínimo, que es menor.
    for (uint64_t semilla = 1; semilla <= 3; ++semilla) {
        Instance in = scenarioInstance(Categoria::LARGE, semilla);
        std::vector<int> nodes = {in.base};
        nodes.insert(nodes.end(), in.clientes.begin(), in.clientes.end());
        double treeCost = primMst(in.dist, nodes).cost;
        double approx = HeuristicSolver::tourCost(in.dist, in.base,
                                                  HeuristicSolver::mstApproximation(in.dist, in.base, in.clientes));
        assert(treeCost <= approx + TOLERANCE && approx <= 2.0 * treeCost + TOLERANCE);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testIteratedLocalSearch() {
    std::cout << "[TEST] 9. Busqueda local iterada: mejora, es reproducible y respeta el tiempo..." << std::endl;

    // Nunca peor que el 2-opt de su ruta inicial, y la misma semilla da la misma ruta.
    for (uint64_t semilla = 1; semilla <= 3; ++semilla) {
        Instance in = scenarioInstance(Categoria::MEDIUM, semilla);
        std::vector<int> start = HeuristicSolver::nearestNeighbor(in.dist, in.base, in.clientes);
        std::vector<int> localOptimum = start;
        HeuristicSolver::twoOpt(in.dist, in.base, localOptimum);

        HeuristicSolver::IlsResult first = HeuristicSolver::iteratedLocalSearch(in.dist, in.base, start, semilla);
        HeuristicSolver::IlsResult second = HeuristicSolver::iteratedLocalSearch(in.dist, in.base, start, semilla);
        assert(first.order == second.order);
        assert(first.iterations == HeuristicSolver::DEFAULT_ILS_ITERATIONS);
        assert(HeuristicSolver::tourCost(in.dist, in.base, first.order) <=
               HeuristicSolver::tourCost(in.dist, in.base, localOptimum) + TOLERANCE);
    }

    // En instancias Small llega al óptimo del branch and bound.
    for (uint64_t semilla = 1; semilla <= 10; ++semilla) {
        Instance in = scenarioInstance(Categoria::SMALL, semilla);
        std::vector<int> start = HeuristicSolver::nearestNeighbor(in.dist, in.base, in.clientes);
        double ils = HeuristicSolver::tourCost(in.dist, in.base,
                                               HeuristicSolver::iteratedLocalSearch(in.dist, in.base, start, semilla).order);
        double optimum = ExactSolver::solve(in.dist, in.base, in.clientes).cost;
        assert(ils <= optimum + TOLERANCE);
    }

    // Con un límite de 30 ms y muchas iteraciones pedidas, se detiene a tiempo.
    Instance large = scenarioInstance(Categoria::LARGE, 2);
    std::vector<int> start = HeuristicSolver::nearestNeighbor(large.dist, large.base, large.clientes);
    auto begin = HeuristicSolver::Clock::now();
    HeuristicSolver::IlsResult limited = HeuristicSolver::iteratedLocalSearch(
        large.dist, large.base, start, 1, 1000000, begin + std::chrono::milliseconds(30));
    double elapsedMs = std::chrono::duration<double, std::milli>(HeuristicSolver::Clock::now() - begin).count();
    assert(limited.stoppedByTime && isPermutationOf(limited.order, large.clientes));
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
