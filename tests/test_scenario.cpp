// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/scenario/Scenario.hpp"
#include "../src/scenario/ScenarioIO.hpp"
#include "../src/geometry/GeometryUtils.hpp"

#include <cassert>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;

// Ejemplo de docs/decisiones.md §6, copiado tal cual.
const std::string DECISIONS_EXAMPLE =
    "NAME small_001\n"
    "CATEGORY SMALL\n"
    "SEED 42\n"
    "AREA 1000 1000\n"
    "BATTERY 450\n"
    "DEPOT 500 500\n"
    "STATIONS 2\n"
    "200 300\n"
    "800 700\n"
    "CLIENTS 3\n"
    "120 880\n"
    "640 150\n"
    "900 420\n"
    "OBSTACLES 2\n"
    "4  300 400  400 400  400 500  300 500\n"
    "3  600 600  700 600  650 690\n";

// Encabezado válido de 6 líneas para armar los casos de error.
const std::string VALID_HEADER =
    "NAME x\nCATEGORY SMALL\nSEED 1\nAREA 100 100\nBATTERY 50\nDEPOT 0 0\n";

Scenario readText(const std::string& text) {
    std::istringstream in(text);
    return ScenarioIO::read(in);
}

bool samePoints(const std::vector<Point>& a, const std::vector<Point>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

void assertSameScenario(const Scenario& a, const Scenario& b) {
    assert(a.name == b.name);
    assert(a.category == b.category);
    assert(a.seed == b.seed);
    assert(a.areaWidth == b.areaWidth && a.areaHeight == b.areaHeight);
    assert(a.battery == b.battery);
    assert(a.depot == b.depot);
    assert(samePoints(a.stations, b.stations));
    assert(samePoints(a.clients, b.clients));
    assert(a.obstacles.size() == b.obstacles.size());
    for (size_t i = 0; i < a.obstacles.size(); ++i) {
        assert(samePoints(a.obstacles[i].vertices, b.obstacles[i].vertices));
    }
}

// Comprueba que el texto produce un error y que el mensaje contiene el fragmento esperado.
void expectError(const std::string& text, const std::string& expectedFragment) {
    try {
        readText(text);
    } catch (const std::runtime_error& error) {
        std::string message = error.what();
        if (message.find(expectedFragment) == std::string::npos) {
            std::cerr << "  Mensaje recibido: " << message << "\n  Se esperaba: " << expectedFragment << std::endl;
            assert(false);
        }
        return;
    }
    std::cerr << "  No hubo error; se esperaba: " << expectedFragment << std::endl;
    assert(false);
}

void testDecisionsExample() {
    std::cout << "[TEST] 1. Lectura del ejemplo de docs/decisiones.md..." << std::endl;

    Scenario s = readText(DECISIONS_EXAMPLE);
    assert(s.name == "small_001");
    assert(s.category == DatasetCategory::SMALL);
    assert(s.seed == 42);
    assert(s.areaWidth == 1000.0 && s.areaHeight == 1000.0);
    assert(s.battery == 450.0);
    assert(s.depot == Point(500, 500));
    assert(s.numStations() == 2 && s.stations[1] == Point(800, 700));
    assert(s.numClients() == 3 && s.clients[2] == Point(900, 420));
    assert(s.numObstacles() == 2);
    assert(s.obstacles[0].size() == 4 && s.obstacles[1].size() == 3);
    assert(s.obstacles[1][2] == Point(650, 690));
    assert(s.totalObstacleVertices() == 7);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testRoundTrip() {
    std::cout << "[TEST] 2. Escribir y volver a leer conserva el escenario..." << std::endl;

    Scenario original = readText(DECISIONS_EXAMPLE);
    original.battery = 450.5;                   // Valor no entero
    original.seed = 18446744073709551615ULL;    // Semilla máxima de 64 bits

    std::stringstream buffer;
    ScenarioIO::write(original, buffer);
    assertSameScenario(original, ScenarioIO::read(buffer));

    // El ejemplo escrito de nuevo debe ser idéntico al texto original.
    std::stringstream rewritten;
    ScenarioIO::write(readText(DECISIONS_EXAMPLE), rewritten);
    assert(rewritten.str() == DECISIONS_EXAMPLE);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testSaveAndLoadFile() {
    std::cout << "[TEST] 3. Guardar en archivo (creando carpetas) y cargar..." << std::endl;

    const std::string folder = "tmp_test_scenario";
    const std::string path = folder + "/sub/escenario.txt";
    std::filesystem::remove_all(folder);

    Scenario original = readText(DECISIONS_EXAMPLE);
    ScenarioIO::save(original, path);
    assertSameScenario(original, ScenarioIO::load(path));
    std::filesystem::remove_all(folder);

    bool failed = false;
    try {
        ScenarioIO::load("no_existe.txt");
    } catch (const std::runtime_error& error) {
        failed = std::string(error.what()).find("no se pudo abrir") != std::string::npos;
    }
    assert(failed);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testErrors() {
    std::cout << "[TEST] 4. Errores con linea y descripcion..." << std::endl;

    expectError("NAME x\nCATEGORY HUGE\n", "linea 2: categoria desconocida 'HUGE'");
    expectError("NAME x\nCATEGORY SMALL\nSEED -3\n", "linea 3: la semilla debe ser un entero no negativo");
    expectError("NAME x\nCATEGORY SMALL\nSEED 1\nAREA 100 100\nBATTERY -5\n",
                "linea 5: la autonomia de la bateria debe ser positiva");
    expectError(VALID_HEADER + "STATION 1\n5 5\nCLIENTS 0\nOBSTACLES 0\n",
                "linea 7: se esperaba la etiqueta STATIONS y se encontro 'STATION'");
    expectError(VALID_HEADER + "STATIONS 1.5\n",
                "linea 7: la cantidad de estaciones debe ser un entero no negativo");
    expectError(VALID_HEADER + "STATIONS 1\n5 cinco\nCLIENTS 0\nOBSTACLES 0\n",
                "linea 8: se esperaba la coordenada y de la estacion 1 (un numero) y se encontro 'cinco'");
    expectError(VALID_HEADER + "STATIONS 0\nCLIENTS 2\n1 1\nOBSTACLES 0\n",
                "linea 10: se esperaba la coordenada x del cliente 2 (un numero) y se encontro 'OBSTACLES'");
    expectError(VALID_HEADER + "STATIONS 0\nCLIENTS 0\nOBSTACLES 1\n2  0 0  1 1\n",
                "linea 10: el obstaculo 1 necesita al menos 3 vertices");
    expectError(VALID_HEADER + "STATIONS 0\nCLIENTS 0\n",
                "linea 8: el archivo termina antes de la etiqueta OBSTACLES");
    expectError(VALID_HEADER + "STATIONS 0\nCLIENTS 0\nOBSTACLES 0\nextra\n",
                "linea 10: contenido inesperado 'extra'");

    std::cout << "  -> PASSED\n" << std::endl;
}

// Ningún punto de interés puede estar dentro ni sobre el borde de un obstáculo.
void assertPointsOutsideObstacles(const Scenario& s) {
    std::vector<Point> points = s.stations;
    points.insert(points.end(), s.clients.begin(), s.clients.end());
    points.push_back(s.depot);
    for (const Point& p : points) {
        for (const Polygon& poly : s.obstacles) {
            assert(GeometryUtils::pointInPolygon(p, poly) == PointLocation::OUTSIDE);
        }
    }
}

bool clear(const Scenario& s, const Point& a, const Point& b) {
    return GeometryUtils::isFlightPathClear(Segment(a, b), s.obstacles);
}

void testAdversarialFiles(const std::string& folder) {
    std::cout << "[TEST] 5. Casos adversariales de " << folder << "..." << std::endl;

    const std::vector<std::string> names = {
        "adv_collinear", "adv_enclosed", "adv_corridor", "adv_concave_trap", "adv_unreachable_battery"
    };
    for (const std::string& name : names) {
        Scenario s = ScenarioIO::load(folder + "/" + name + ".txt");
        assert(s.name == name);
        assert(s.category == DatasetCategory::ADVERSARIAL);
        assert(s.numClients() > 0);
        assertPointsOutsideObstacles(s);
    }

    // Comprobaciones geométricas de cada caso, para confirmar que miden lo que dicen.
    Scenario collinear = ScenarioIO::load(folder + "/adv_collinear.txt");
    assert(clear(collinear, collinear.depot, collinear.clients[0]));          // Vuela sobre el borde inferior
    assert(!clear(collinear, collinear.stations[0], collinear.clients[3]));   // x = 50 cruza el obstáculo

    Scenario enclosed = ScenarioIO::load(folder + "/adv_enclosed.txt");
    Point trapped = enclosed.clients[0];
    const std::vector<Point> outside = {enclosed.depot, enclosed.stations[0], enclosed.clients[1], enclosed.clients[2],
                                        Point(50, 0), Point(50, 100), Point(0, 50), Point(100, 50)};
    for (const Point& p : outside) {
        assert(!clear(enclosed, trapped, p));                                  // Ninguna salida directa
    }

    Scenario corridor = ScenarioIO::load(folder + "/adv_corridor.txt");
    assert(clear(corridor, corridor.depot, corridor.clients[0]));             // Recto por el pasillo

    Scenario trap = ScenarioIO::load(folder + "/adv_concave_trap.txt");
    assert(clear(trap, trap.depot, trap.clients[0]));                         // Entra al hueco de la U
    assert(!clear(trap, trap.depot, trap.clients[1]));                        // El cliente de atrás está bloqueado

    std::cout << "  -> PASSED\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string adversarialFolder = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DE ESCENARIOS (TXT)      " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    testDecisionsExample();
    testRoundTrip();
    testSaveAndLoadFile();
    testErrors();
    testAdversarialFiles(adversarialFolder);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DE ESCENARIOS PASARON CON EXITO     " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
