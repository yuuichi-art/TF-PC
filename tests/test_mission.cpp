// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/solvers/ExactSolver.hpp"
#include "../src/solvers/MissionSolver.hpp"
#include "../src/report/Exporter.hpp"
#include "../src/scenario/DatasetGenerator.hpp"
#include "../src/scenario/ScenarioIO.hpp"
#include "../src/geometry/GeometryUtils.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;

const double TOLERANCE = 1e-6;

bool isRechargeLabel(const std::string& label) { return label[0] == 'D' || label[0] == 'S'; }

// Verificación independiente del solver, solo con la ruta y el escenario: lo mismo que hará el juez.
void assertValidRoute(const Scenario& s, const Route& route, size_t expectedClients) {
    assert(route.feasible);
    assert(!route.path.empty() && route.path.front().label == "D0" && route.path.back().label == "D0");
    assert(route.order.front() == "D0" && route.order.back() == "D0");
    assert(route.preparationMs >= 0 && route.solveMs >= 0);

    double battery = s.battery;
    double length = 0.0;
    int recharges = 0;
    std::set<std::string> visited;
    std::vector<std::string> keyLabels = {route.path.front().label};
    for (size_t i = 1; i < route.path.size(); ++i) {
        Segment leg(route.path[i - 1].point, route.path[i].point);
        assert(GeometryUtils::isFlightPathClear(leg, s.obstacles));   // Ningún tramo cruza un obstáculo
        length += leg.length();
        battery -= leg.length();
        assert(battery >= -TOLERANCE);                                 // La batería nunca baja de 0

        const std::string& label = route.path[i].label;
        if (label == "V") continue;
        keyLabels.push_back(label);
        if (isRechargeLabel(label)) {
            battery = s.battery;
            if (i + 1 < route.path.size()) ++recharges;
        } else {
            assert(visited.insert(label).second);                      // Cada cliente una sola vez
        }
    }
    assert(std::abs(length - route.cost) < TOLERANCE);
    assert(recharges == route.recharges);
    assert(visited.size() == expectedClients);
    assert(keyLabels == route.order);                                  // La trayectoria sigue el orden
}

Scenario handScenario() {
    // Base (0,0), estación (10,0), cliente (10,6), batería 12: la ruta óptima es
    // D0 -> S1 -> C1 -> S1 -> D0 = 32, con dos recargas (calculada a mano en test_network).
    Scenario s;
    s.name = "hand";
    s.category = DatasetCategory::ADVERSARIAL;
    s.battery = 12;
    s.depot = Point(0, 0);
    s.stations = {Point(10, 0)};
    s.clients = {Point(10, 6)};
    return s;
}

void testHandRouteAndExport() {
    std::cout << "[TEST] 1. Ruta calculada a mano y su archivo exportado..." << std::endl;

    Scenario s = handScenario();
    MissionSolver solver(s);
    Route route = solver.solve(Algorithm::BRANCH_AND_BOUND);
    assertValidRoute(s, route, 1);
    assert(route.order == std::vector<std::string>({"D0", "S1", "C1", "S1", "D0"}));
    assert(std::abs(route.cost - 32.0) < TOLERANCE && route.recharges == 2);

    std::ostringstream out;
    Exporter::writeRoute(route, out);
    const std::string expected =
        "SCENARIO hand\n"
        "ALGORITHM branch_and_bound\n"
        "STATUS OK\n"
        "COST 32.000000\n"
        "RECHARGES 2\n"
        "EXCLUDED 0\n"
        "ORDER 5\n"
        "D0 S1 C1 S1 D0\n"
        "PATH 5\n"
        "0 0 D0\n"
        "10 0 S1\n"
        "10 6 C1\n"
        "10 0 S1\n"
        "0 0 D0\n";
    assert(out.str() == expected);

    // saveRoute crea la carpeta y escribe lo mismo.
    const std::string folder = "tmp_test_mission";
    std::filesystem::remove_all(folder);
    Exporter::saveRoute(route, folder + "/ruta.txt");
    std::ifstream file(folder + "/ruta.txt");
    std::stringstream content;
    content << file.rdbuf();
    file.close();
    assert(content.str() == expected);
    std::filesystem::remove_all(folder);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testAllAlgorithmsProduceValidRoutes(const std::string& folder) {
    std::cout << "[TEST] 2. Todos los algoritmos dan rutas validas (adversariales y generados)..." << std::endl;

    std::vector<Scenario> scenarios;
    for (const char* name : {"adv_collinear", "adv_enclosed", "adv_corridor", "adv_concave_trap",
                             "adv_unreachable_battery"}) {
        scenarios.push_back(ScenarioIO::load(folder + "/" + name + ".txt"));
    }
    for (uint64_t seed = 1; seed <= 2; ++seed) {
        scenarios.push_back(DatasetGenerator::generate(DatasetCategory::SMALL, seed));
        scenarios.push_back(DatasetGenerator::generate(DatasetCategory::MEDIUM, seed));
    }

    for (const Scenario& s : scenarios) {
        MissionSolver solver(s);
        assert(solver.servableClients().size() + solver.excluded().size() == static_cast<size_t>(s.numClients()));
        for (Algorithm algorithm : allAlgorithms()) {
            if (algorithm == Algorithm::BRANCH_AND_BOUND && !solver.supportsExact()) continue;
            Route route = solver.solve(algorithm);
            assert(route.algorithm == algorithmName(algorithm) && route.scenario == s.name);
            assertValidRoute(s, route, solver.servableClients().size());
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testExclusions(const std::string& folder) {
    std::cout << "[TEST] 3. Clientes excluidos por obstaculos y por bateria..." << std::endl;

    Scenario enclosed = ScenarioIO::load(folder + "/adv_enclosed.txt");
    MissionSolver enclosedSolver(enclosed);
    assert(enclosedSolver.excluded().size() == 1);
    assert(enclosedSolver.excluded()[0].label == "C1" && enclosedSolver.excluded()[0].reason == "OBSTACLES");
    assertValidRoute(enclosed, enclosedSolver.solve(Algorithm::BRANCH_AND_BOUND), 2);

    Scenario battery = ScenarioIO::load(folder + "/adv_unreachable_battery.txt");
    MissionSolver batterySolver(battery);
    assert(batterySolver.excluded().size() == 1);
    assert(batterySolver.excluded()[0].label == "C4" && batterySolver.excluded()[0].reason == "BATTERY");
    Route route = batterySolver.solve(Algorithm::ITERATED_LOCAL_SEARCH);
    assertValidRoute(battery, route, 3);
    assert(route.excluded.size() == 1);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testExactAvailability() {
    std::cout << "[TEST] 4. El exacto solo se ofrece con pocos clientes..." << std::endl;

    Scenario medium = DatasetGenerator::generate(DatasetCategory::MEDIUM, 1);
    assert(medium.numClients() > ExactSolver::MAX_CLIENTS);
    MissionSolver solver(medium);
    assert(!solver.supportsExact());
    bool rejected = false;
    try {
        solver.solve(Algorithm::BRANCH_AND_BOUND);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testNoClients() {
    std::cout << "[TEST] 5. Sin clientes el dron no despega..." << std::endl;

    Scenario s = handScenario();
    s.clients.clear();
    MissionSolver solver(s);
    for (Algorithm algorithm : allAlgorithms()) {
        Route route = solver.solve(algorithm);
        assert(route.feasible && route.cost == 0.0 && route.recharges == 0);
        assert(route.order == std::vector<std::string>{"D0"} && route.path.size() == 1);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

size_t countOf(const std::string& text, const std::string& fragment) {
    size_t count = 0;
    for (size_t pos = text.find(fragment); pos != std::string::npos; pos = text.find(fragment, pos + 1)) ++count;
    return count;
}

std::string svgOf(const Scenario& s, const Route& route) {
    std::ostringstream out;
    Exporter::writeSvg(s, route, out);
    return out.str();
}

void testSvg(const std::string& folder) {
    std::cout << "[TEST] 6. Dibujo SVG del escenario y la ruta..." << std::endl;

    // Ruta a mano: un SVG completo con un elemento de cada tipo y la trayectoria de 5 puntos.
    Scenario hand = handScenario();
    MissionSolver handSolver(hand);
    std::string svg = svgOf(hand, handSolver.solve(Algorithm::BRANCH_AND_BOUND));
    assert(svg.rfind("<svg xmlns=\"http://www.w3.org/2000/svg\"", 0) == 0);
    assert(svg.size() >= 7 && svg.compare(svg.size() - 7, 7, "</svg>\n") == 0);
    assert(countOf(svg, "class=\"route\"") == 1 && countOf(svg, "class=\"depot\"") == 1);
    assert(countOf(svg, "class=\"station\"") == 1 && countOf(svg, "class=\"client\"") == 1);
    assert(countOf(svg, "class=\"obstacle\"") == 0 && countOf(svg, "class=\"excluded\"") == 0);
    size_t start = svg.find("points=\"", svg.find("class=\"route\"")) + 8;
    std::string points = svg.substr(start, svg.find('"', start) - start);
    assert(std::count(points.begin(), points.end(), ' ') + 1 == 5);
    // Área de 1000 x 1000 en 800 px (escala 0,8), margen 20 y encabezado 40: la base (0,0) queda
    // en el píxel (20, 840) y su cuadrado de 12 px empieza en (14, 834). El eje y se invierte.
    assert(svg.find("class=\"depot\" x=\"14.00\" y=\"834.00\"") != std::string::npos);
    assert(svg.find(">C1</text>") != std::string::npos && svg.find("costo 32.00") != std::string::npos);

    // Cliente encerrado: 4 obstáculos, el excluido como cruz y los otros dos como círculos.
    Scenario enclosed = ScenarioIO::load(folder + "/adv_enclosed.txt");
    MissionSolver enclosedSolver(enclosed);
    svg = svgOf(enclosed, enclosedSolver.solve(Algorithm::BRANCH_AND_BOUND));
    assert(countOf(svg, "class=\"obstacle\"") == 4);
    assert(countOf(svg, "class=\"excluded\"") == 1 && countOf(svg, "class=\"client\"") == 2);

    // Con más de 40 clientes se omiten las etiquetas para que el dibujo se lea.
    Scenario large = DatasetGenerator::generate(DatasetCategory::LARGE, 1);
    MissionSolver largeSolver(large);
    svg = svgOf(large, largeSolver.solve(Algorithm::NEAREST_NEIGHBOR));
    assert(countOf(svg, "class=\"client\"") == static_cast<size_t>(large.numClients()));
    assert(svg.find(">C1</text>") == std::string::npos);

    // saveSvg crea la carpeta y el archivo.
    const std::string folderOut = "tmp_test_svg";
    std::filesystem::remove_all(folderOut);
    Exporter::saveSvg(hand, handSolver.solve(Algorithm::NEAREST_NEIGHBOR), folderOut + "/ruta.svg");
    assert(std::filesystem::file_size(folderOut + "/ruta.svg") > 500);
    std::filesystem::remove_all(folderOut);

    std::cout << "  -> PASSED\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string adversarialFolder = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DE LA MISION COMPLETA    " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    testHandRouteAndExport();
    testAllAlgorithmsProduceValidRoutes(adversarialFolder);
    testExclusions(adversarialFolder);
    testExactAvailability();
    testNoClients();
    testSvg(adversarialFolder);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DE LA MISION PASARON CON EXITO      " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
