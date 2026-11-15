#include "DatasetGenerator.hpp"
#include "ScenarioIO.hpp"
#include "../geometry/ConvexHull.hpp"
#include "../geometry/GeometryUtils.hpp"
#include "../graph/ChargingNetwork.hpp"
#include "../graph/MissionGraph.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <utility>

namespace geodrone {

namespace {

// Entero uniforme en [lo, hi]. Se usa el módulo en lugar de std::uniform_int_distribution
// porque la salida de mt19937_64 está fijada por el estándar y la de la distribución no:
// así una semilla produce el mismo escenario con cualquier compilador.
// El sesgo del módulo es despreciable con rangos de miles frente a 2^64.
long long randomInt(std::mt19937_64& rng, long long lo, long long hi) {
    unsigned long long range = static_cast<unsigned long long>(hi - lo) + 1ULL;
    return lo + static_cast<long long>(rng() % range);
}

// Obstáculos convexos sin solape. El área se divide en una cuadrícula de side x side celdas
// (unas el doble que obstáculos, para dejar espacio libre) y cada obstáculo ocupa una celda
// distinta elegida al azar. Dentro de la celda se deja un margen del 15 % por lado, así que
// dos obstáculos nunca se tocan.
std::vector<Polygon> generateObstacles(std::mt19937_64& rng, int count, int maxPoints,
                                       double width, double height) {
    std::vector<Polygon> obstacles;
    if (count == 0) return obstacles;

    int side = std::max(3, static_cast<int>(std::ceil(std::sqrt(2.0 * count))));
    double cellWidth = width / side;
    double cellHeight = height / side;

    // Orden aleatorio de las celdas (Fisher-Yates).
    std::vector<int> cells(side * side);
    std::iota(cells.begin(), cells.end(), 0);
    for (int i = static_cast<int>(cells.size()) - 1; i > 0; --i) {
        std::swap(cells[i], cells[randomInt(rng, 0, i)]);
    }

    obstacles.reserve(count);
    for (int k = 0; k < count; ++k) {
        int column = cells[k] % side;
        int row = cells[k] / side;
        long long minX = static_cast<long long>(std::ceil((column + 0.15) * cellWidth));
        long long maxX = static_cast<long long>(std::floor((column + 0.85) * cellWidth));
        long long minY = static_cast<long long>(std::ceil((row + 0.15) * cellHeight));
        long long maxY = static_cast<long long>(std::floor((row + 0.85) * cellHeight));

        // Se repite en el caso raro de puntos colineales (envolvente con menos de 3 vértices).
        std::vector<Point> hull;
        while (hull.size() < 3) {
            int pointCount = static_cast<int>(randomInt(rng, 3, maxPoints));
            std::vector<Point> points;
            points.reserve(pointCount);
            for (int p = 0; p < pointCount; ++p) {
                points.emplace_back(static_cast<double>(randomInt(rng, minX, maxX)),
                                    static_cast<double>(randomInt(rng, minY, maxY)));
            }
            hull = convexHull(points);
        }
        obstacles.emplace_back(k + 1, hull, "Obs" + std::to_string(k + 1));
    }
    return obstacles;
}

// Punto entero libre: fuera de todo obstáculo (ni dentro ni sobre el borde) y no usado antes.
Point randomFreePoint(std::mt19937_64& rng, double width, double height,
                      const std::vector<Polygon>& obstacles,
                      std::set<std::pair<long long, long long>>& used) {
    while (true) {
        long long x = randomInt(rng, 0, static_cast<long long>(width));
        long long y = randomInt(rng, 0, static_cast<long long>(height));
        if (used.count({x, y})) continue;

        Point candidate(static_cast<double>(x), static_cast<double>(y));
        bool free = true;
        for (const Polygon& poly : obstacles) {
            if (GeometryUtils::pointInPolygon(candidate, poly) != PointLocation::OUTSIDE) {
                free = false;
                break;
            }
        }
        if (free) {
            used.insert({x, y});
            return candidate;
        }
    }
}

std::string lowercase(std::string text) {
    for (char& c : text) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return text;
}

} // namespace

CategorySpec DatasetGenerator::specFor(DatasetCategory category) {
    // Clientes y obstáculos según la tabla del README. Puntos por obstáculo elegidos para
    // respetar el máximo de vértices de obstáculos de cada categoría (30, 100, 400 y 1500).
    switch (category) {
        case DatasetCategory::SMALL:   return {1, 10, 1, 3, 1, 5, 6};
        case DatasetCategory::MEDIUM:  return {11, 40, 3, 6, 5, 15, 6};
        case DatasetCategory::LARGE:   return {41, 200, 6, 12, 15, 50, 8};
        case DatasetCategory::EXTREME: return {201, 1000, 12, 25, 50, 150, 10};
        case DatasetCategory::ADVERSARIAL: break;
    }
    throw std::invalid_argument("la categoria ADVERSARIAL no se genera: sus casos se escriben a mano");
}

Scenario DatasetGenerator::generate(DatasetCategory category, uint64_t seed, double width, double height) {
    Scenario scenario = generateLayout(specFor(category), category, seed, width, height);
    scenario.battery = batteryFor(scenario);
    return scenario;
}

double DatasetGenerator::batteryFor(const Scenario& layout) {
    MissionGraph mission(layout);
    double minimum = ChargingNetwork::minimumAutonomy(mission);
    return std::max(1.0, std::ceil(minimum * BATTERY_MARGIN));
}

Scenario DatasetGenerator::generateLayout(const CategorySpec& spec, DatasetCategory category, uint64_t seed,
                                          double width, double height) {
    std::mt19937_64 rng(seed);

    Scenario scenario;
    scenario.category = category;
    scenario.seed = seed;
    scenario.name = lowercase(categoryName(category)) + "_" + std::to_string(seed);
    scenario.areaWidth = width;
    scenario.areaHeight = height;
    scenario.battery = 0.0;   // Sin calcular: lo completa batteryFor

    // El orden de los sorteos es fijo: cambiarlo cambia los escenarios de cada semilla.
    int clientCount = static_cast<int>(randomInt(rng, spec.minClients, spec.maxClients));
    int stationCount = static_cast<int>(randomInt(rng, spec.minStations, spec.maxStations));
    int obstacleCount = static_cast<int>(randomInt(rng, spec.minObstacles, spec.maxObstacles));

    scenario.obstacles = generateObstacles(rng, obstacleCount, spec.maxPointsPerObstacle, width, height);

    std::set<std::pair<long long, long long>> used;
    scenario.depot = randomFreePoint(rng, width, height, scenario.obstacles, used);
    for (int i = 0; i < stationCount; ++i) {
        scenario.stations.push_back(randomFreePoint(rng, width, height, scenario.obstacles, used));
    }
    for (int i = 0; i < clientCount; ++i) {
        scenario.clients.push_back(randomFreePoint(rng, width, height, scenario.obstacles, used));
    }
    return scenario;
}

std::vector<std::string> DatasetGenerator::generateBenchmarkSet(const std::string& folder, int seedsPerCategory) {
    const DatasetCategory categories[] = {
        DatasetCategory::SMALL, DatasetCategory::MEDIUM, DatasetCategory::LARGE, DatasetCategory::EXTREME
    };
    std::vector<std::string> paths;
    for (DatasetCategory category : categories) {
        for (int seed = 1; seed <= seedsPerCategory; ++seed) {
            Scenario scenario = generate(category, static_cast<uint64_t>(seed));
            std::string path = folder + "/" + scenario.name + ".txt";
            ScenarioIO::save(scenario, path);
            paths.push_back(path);
        }
    }
    return paths;
}

} // namespace geodrone
