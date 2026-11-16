// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/escenario/GeneradorDatasets.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"
#include "../src/geometria/Geometria.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;

const DatasetCategory GENERATED_CATEGORIES[] = {
    DatasetCategory::SMALL, DatasetCategory::MEDIUM, DatasetCategory::LARGE, DatasetCategory::EXTREME
};

std::string toText(const Scenario& scenario) {
    std::ostringstream out;
    ScenarioIO::write(scenario, out);
    return out.str();
}

// Solo la geometría: las pruebas de este archivo no necesitan la batería, que construye el
// grafo de misión completo y haría lentas las pruebas con muchos escenarios Extreme.
Scenario layout(DatasetCategory category, uint64_t seed) {
    return DatasetGenerator::generateLayout(DatasetGenerator::specFor(category), category, seed);
}

bool isInteger(double value) {
    return value == std::floor(value);
}

bool insideArea(const Scenario& s, const Point& p) {
    return p.x >= 0 && p.x <= s.areaWidth && p.y >= 0 && p.y <= s.areaHeight;
}

// Dos polígonos convexos se tocan o se solapan si alguna arista de uno corta (o toca) una
// arista del otro, o si un vértice de uno está dentro o sobre el borde del otro.
bool polygonsTouch(const Polygon& a, const Polygon& b) {
    for (const Segment& ea : a.getEdges()) {
        for (const Segment& eb : b.getEdges()) {
            if (GeometryUtils::segmentsIntersect(ea, eb)) return true;
        }
    }
    return GeometryUtils::pointInPolygon(a[0], b) != PointLocation::OUTSIDE ||
           GeometryUtils::pointInPolygon(b[0], a) != PointLocation::OUTSIDE;
}

void testSameSeedSameScenario() {
    std::cout << "[TEST] 1. La misma semilla produce el mismo escenario..." << std::endl;

    for (DatasetCategory category : GENERATED_CATEGORIES) {
        for (uint64_t seed : {1ULL, 7ULL, 123456789ULL}) {
            std::string first = toText(layout(category, seed));
            std::string second = toText(layout(category, seed));
            assert(first == second);
        }
        // Semillas distintas dan escenarios distintos.
        assert(toText(layout(category, 1)) != toText(layout(category, 2)));
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testSizesWithinCategory() {
    std::cout << "[TEST] 2. Cantidades dentro de los rangos de cada categoria..." << std::endl;

    const int maxObstacleVertices[] = {30, 100, 400, 1500};
    int index = 0;
    for (DatasetCategory category : GENERATED_CATEGORIES) {
        CategorySpec spec = DatasetGenerator::specFor(category);
        for (uint64_t seed = 1; seed <= 10; ++seed) {
            Scenario s = layout(category, seed);
            assert(s.category == category && s.seed == seed);
            assert(s.numClients() >= spec.minClients && s.numClients() <= spec.maxClients);
            assert(s.numStations() >= spec.minStations && s.numStations() <= spec.maxStations);
            assert(s.numObstacles() >= spec.minObstacles && s.numObstacles() <= spec.maxObstacles);
            assert(s.totalObstacleVertices() <= maxObstacleVertices[index]);
            for (const Polygon& poly : s.obstacles) {
                assert(poly.size() >= 3 && poly.size() <= spec.maxPointsPerObstacle);
            }
        }
        ++index;
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testGeometryIsValid() {
    std::cout << "[TEST] 3. Obstaculos convexos sin solape y puntos libres y enteros..." << std::endl;

    for (DatasetCategory category : GENERATED_CATEGORIES) {
        for (uint64_t seed = 1; seed <= 5; ++seed) {
            Scenario s = layout(category, seed);

            // Obstáculos: convexos (antihorarios, sin giros a la derecha), dentro del área,
            // con coordenadas enteras y sin tocarse entre sí.
            for (const Polygon& poly : s.obstacles) {
                int n = poly.size();
                for (int i = 0; i < n; ++i) {
                    const Point& v = poly[i];
                    assert(isInteger(v.x) && isInteger(v.y) && insideArea(s, v));
                    assert(GeometryUtils::ccw(poly[i], poly[(i + 1) % n], poly[(i + 2) % n]) > 0);
                }
            }
            for (int i = 0; i < s.numObstacles(); ++i) {
                for (int j = i + 1; j < s.numObstacles(); ++j) {
                    assert(!polygonsTouch(s.obstacles[i], s.obstacles[j]));
                }
            }

            // Base, estaciones y clientes: enteros, dentro del área, fuera de los obstáculos
            // y sin repetir ningún punto.
            std::vector<Point> points = s.stations;
            points.insert(points.end(), s.clients.begin(), s.clients.end());
            points.push_back(s.depot);
            for (size_t i = 0; i < points.size(); ++i) {
                assert(isInteger(points[i].x) && isInteger(points[i].y) && insideArea(s, points[i]));
                for (const Polygon& poly : s.obstacles) {
                    assert(GeometryUtils::pointInPolygon(points[i], poly) == PointLocation::OUTSIDE);
                }
                for (size_t j = i + 1; j < points.size(); ++j) {
                    assert(points[i] != points[j]);
                }
            }
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testAdversarialIsRejected() {
    std::cout << "[TEST] 4. La categoria ADVERSARIAL no se genera..." << std::endl;

    bool rejected = false;
    try {
        DatasetGenerator::generate(DatasetCategory::ADVERSARIAL, 1);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testBenchmarkSet() {
    std::cout << "[TEST] 5. Set de benchmark guardado y releido desde archivo..." << std::endl;

    const std::string folder = "tmp_test_generator";
    std::filesystem::remove_all(folder);

    // Una semilla por categoría y sin el peor caso: cada escenario completo construye su grafo
    // de misión, y el Extreme al máximo tarda varios segundos (más aún en Debug).
    std::vector<std::string> paths = DatasetGenerator::generateBenchmarkSet(folder, 1, false);
    assert(paths.size() == 4);
    assert(paths.front() == folder + "/small_1.txt");
    assert(paths.back() == folder + "/extreme_1.txt");

    // Lo guardado es exactamente lo que genera la semilla, y tiene batería calculada.
    Scenario fromFile = ScenarioIO::load(folder + "/medium_1.txt");
    assert(toText(fromFile) == toText(DatasetGenerator::generate(DatasetCategory::MEDIUM, 1)));
    assert(fromFile.battery >= 1.0 && isInteger(fromFile.battery));

    // La geometría del escenario completo es la misma que la de generateLayout.
    Scenario full = DatasetGenerator::generate(DatasetCategory::SMALL, 3);
    Scenario onlyLayout = layout(DatasetCategory::SMALL, 3);
    onlyLayout.battery = full.battery;
    assert(toText(full) == toText(onlyLayout));

    // El peor caso fija cada cantidad en el máximo del rango.
    for (DatasetCategory category : GENERATED_CATEGORIES) {
        CategorySpec range = DatasetGenerator::specFor(category);
        Scenario worst = DatasetGenerator::generateLayout(DatasetGenerator::worstCaseSpecFor(category), category, 1);
        assert(worst.numClients() == range.maxClients);
        assert(worst.numStations() == range.maxStations);
        assert(worst.numObstacles() == range.maxObstacles);
    }

    std::filesystem::remove_all(folder);

    std::cout << "  -> PASSED\n" << std::endl;
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DEL GENERADOR            " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    testSameSeedSameScenario();
    testSizesWithinCategory();
    testGeometryIsValid();
    testAdversarialIsRejected();
    testBenchmarkSet();

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DEL GENERADOR PASARON CON EXITO     " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
