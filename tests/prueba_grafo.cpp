// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/grafo/GrafoMision.hpp"
#include "../src/grafo/Dijkstra.hpp"
#include "../src/escenario/GeneradorDatasets.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"
#include "../src/geometria/Geometria.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace geodrone;

const double TOLERANCE = 1e-6;

// Escenarios de prueba: generados (Small y Medium) y los adversariales hechos a mano.
std::vector<Scenario> sampleScenarios(const std::string& adversarialFolder) {
    std::vector<Scenario> scenarios;
    for (uint64_t seed = 1; seed <= 3; ++seed) {
        scenarios.push_back(DatasetGenerator::generate(DatasetCategory::SMALL, seed));
        scenarios.push_back(DatasetGenerator::generate(DatasetCategory::MEDIUM, seed));
    }
    for (const char* name : {"adv_collinear", "adv_enclosed", "adv_corridor", "adv_concave_trap",
                             "adv_unreachable_battery"}) {
        scenarios.push_back(ScenarioIO::load(adversarialFolder + "/" + name + ".txt"));
    }
    return scenarios;
}

VisibilityGraph buildGraph(const Scenario& s, bool useFilter) {
    VisibilityGraph g;
    g.addNode(s.depot, NodeType::DEPOT, "D0");
    for (const Point& p : s.stations) g.addNode(p, NodeType::CHARGING_STATION, "S");
    for (const Point& p : s.clients) g.addNode(p, NodeType::CLIENT, "C");
    for (const Polygon& poly : s.obstacles) g.addObstacle(poly);
    g.buildVisibilityEdges(useFilter);
    return g;
}

std::vector<int> sortedNeighbors(const VisibilityGraph& g, int u) {
    std::vector<int> neighbors;
    for (const GraphEdge& e : g.getNeighbors(u)) neighbors.push_back(e.to);
    std::sort(neighbors.begin(), neighbors.end());
    return neighbors;
}

void testBoundingBoxFilter(const std::vector<Scenario>& scenarios) {
    std::cout << "[TEST] 1. El filtro por caja envolvente da las mismas aristas..." << std::endl;

    for (const Scenario& s : scenarios) {
        VisibilityGraph filtered = buildGraph(s, true);
        VisibilityGraph full = buildGraph(s, false);
        assert(filtered.numEdges() == full.numEdges());
        for (int u = 0; u < full.numVertices(); ++u) {
            assert(sortedNeighbors(filtered, u) == sortedNeighbors(full, u));
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testKeyNodesAsEndpoints(const std::vector<Scenario>& scenarios) {
    std::cout << "[TEST] 2. Dijkstra sin expandir nodos clave da las mismas distancias..." << std::endl;

    for (const Scenario& s : scenarios) {
        MissionGraph mission(s);
        const VisibilityGraph& g = mission.graph();
        for (int a = 0; a < mission.numKeyNodes(); ++a) {
            DijkstraResult plain = Dijkstra::run(g, a, false);
            DijkstraResult endpoints = Dijkstra::run(g, a, true);
            for (int v = 0; v < g.numVertices(); ++v) {
                bool bothInfinite = std::isinf(plain.dist[v]) && std::isinf(endpoints.dist[v]);
                assert(bothInfinite || std::abs(plain.dist[v] - endpoints.dist[v]) < TOLERANCE);
            }
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testMatrixProperties(const std::vector<Scenario>& scenarios) {
    std::cout << "[TEST] 3. Matriz simetrica, diagonal cero y desigualdad triangular..." << std::endl;

    for (const Scenario& s : scenarios) {
        MissionGraph mission(s);
        int k = mission.numKeyNodes();
        assert(k == 1 + s.numStations() + s.numClients());
        for (int a = 0; a < k; ++a) {
            assert(mission.distance(a, a) == 0.0);
            for (int b = 0; b < k; ++b) {
                double ab = mission.distance(a, b);
                double ba = mission.distance(b, a);
                assert((std::isinf(ab) && std::isinf(ba)) || std::abs(ab - ba) < TOLERANCE);
                // Nunca más corto que la línea recta.
                assert(ab >= mission.keyPoint(a).dist(mission.keyPoint(b)) - TOLERANCE);
                for (int c = 0; c < k; ++c) {
                    double viaB = ab + mission.distance(b, c);
                    assert(mission.distance(a, c) <= viaB + TOLERANCE);
                }
            }
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testPaths(const std::vector<Scenario>& scenarios) {
    std::cout << "[TEST] 4. Caminos reales: validos, solo doblan en obstaculos y miden lo de la matriz..." << std::endl;

    for (const Scenario& s : scenarios) {
        MissionGraph mission(s);
        const VisibilityGraph& g = mission.graph();
        for (int a = 0; a < mission.numKeyNodes(); ++a) {
            for (int b = 0; b < mission.numKeyNodes(); ++b) {
                std::vector<int> path = mission.pathNodes(a, b);
                if (std::isinf(mission.distance(a, b))) {
                    assert(path.empty());
                    continue;
                }
                assert(path.front() == a && path.back() == b);
                double length = 0.0;
                for (size_t i = 0; i + 1 < path.size(); ++i) {
                    if (i > 0) assert(g.getNode(path[i]).type == NodeType::OBSTACLE_VERTEX);
                    Segment leg(g.getNode(path[i]).pt, g.getNode(path[i + 1]).pt);
                    assert(GeometryUtils::isFlightPathClear(leg, s.obstacles));
                    length += leg.length();
                }
                assert(std::abs(length - mission.distance(a, b)) < TOLERANCE);
            }
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testHandComputedDistance() {
    std::cout << "[TEST] 5. Distancia calculada a mano alrededor de un cuadrado..." << std::endl;

    // Base (0,5) y cliente (10,5) con el cuadrado [3,7]x[3,7] en medio. El camino mínimo rodea
    // el cuadrado por abajo: (0,5) -> (3,3) -> (7,3) -> (10,5) = 2·sqrt(13) + 4.
    Scenario s;
    s.depot = Point(0, 5);
    s.clients = {Point(10, 5)};
    s.obstacles = {Polygon(1, {Point(3, 3), Point(7, 3), Point(7, 7), Point(3, 7)})};
    MissionGraph mission(s);
    assert(std::abs(mission.distance(0, mission.clientKey(0)) - (2.0 * std::sqrt(13.0) + 4.0)) < TOLERANCE);
    assert(mission.pathNodes(0, mission.clientKey(0)).size() == 4);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testAdversarialCases(const std::string& folder) {
    std::cout << "[TEST] 6. Casos adversariales: DFS, pasillo, borde y trampa concava..." << std::endl;

    // Cliente encerrado: el DFS desde la base no lo alcanza y no hay distancia finita.
    MissionGraph enclosed(ScenarioIO::load(folder + "/adv_enclosed.txt"));
    assert(enclosed.unreachableClients() == std::vector<int>{0});
    assert(!enclosed.isConnectedToDepot(enclosed.clientKey(0)));
    assert(std::isinf(enclosed.distance(0, enclosed.clientKey(0))));
    assert(enclosed.isConnectedToDepot(enclosed.clientKey(1)));
    assert(enclosed.isConnectedToDepot(enclosed.stationKey(0)));

    // Pasillo de ancho 2: la ruta va en línea recta de (10,50) a (90,50).
    MissionGraph corridor(ScenarioIO::load(folder + "/adv_corridor.txt"));
    assert(std::abs(corridor.distance(0, corridor.clientKey(0)) - 80.0) < TOLERANCE);
    assert(corridor.unreachableClients().empty());

    // Vuelo sobre la línea del borde inferior: de (10,30) a (90,30) en línea recta.
    MissionGraph collinear(ScenarioIO::load(folder + "/adv_collinear.txt"));
    assert(std::abs(collinear.distance(0, collinear.clientKey(0)) - 80.0) < TOLERANCE);

    // Trampa en U: el cliente del hueco se alcanza recto; el de atrás obliga a rodear.
    MissionGraph trap(ScenarioIO::load(folder + "/adv_concave_trap.txt"));
    assert(std::abs(trap.distance(0, trap.clientKey(0)) - 35.0) < TOLERANCE);
    double behind = trap.distance(0, trap.clientKey(1));
    assert(!std::isinf(behind) && behind > 70.0 + 1.0);

    std::cout << "  -> PASSED\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string adversarialFolder = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DEL GRAFO DE MISION      " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    std::vector<Scenario> scenarios = sampleScenarios(adversarialFolder);
    testBoundingBoxFilter(scenarios);
    testKeyNodesAsEndpoints(scenarios);
    testMatrixProperties(scenarios);
    testPaths(scenarios);
    testHandComputedDistance();
    testAdversarialCases(adversarialFolder);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DEL GRAFO PASARON CON EXITO         " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
