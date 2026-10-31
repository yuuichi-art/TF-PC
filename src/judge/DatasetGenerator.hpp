#ifndef GEODRONE_DATASETGENERATOR_HPP
#define GEODRONE_DATASETGENERATOR_HPP

#include "../geometry/Point.hpp"
#include "../geometry/Polygon.hpp"
#include <vector>
#include <string>
#include <random>
#include <cstdint>

namespace geodrone {

/**
 * @brief Categorías oficiales de evaluación según la Tabla 1 del diseño técnico.
 */
enum class DatasetCategory {
    SMALL,      // 1 <= N <= 10, 1 <= M <= 5, TL = 1.0s, ML = 256MB
    MEDIUM,     // 11 <= N <= 40, 5 <= M <= 15, TL = 2.0s, ML = 256MB
    LARGE,      // 41 <= N <= 200, 15 <= M <= 50, TL = 3.0s, ML = 512MB
    EXTREME     // 201 <= N <= 1000, 50 <= M <= 150, TL = 5.0s, ML = 512MB
};

/**
 * @brief Representación estructurada de una instancia o escenario operacional.
 */
struct Scenario {
    std::string name;
    DatasetCategory category;
    uint64_t seed;
    Point depot;
    std::vector<Point> clients;
    std::vector<Point> stations;
    std::vector<Polygon> obstacles;
    double maxBattery = 1000.0;
    double timeLimitSeconds = 1.0;
    double memoryLimitMB = 256.0;

    int numClients() const { return static_cast<int>(clients.size()); }
    int numObstacles() const { return static_cast<int>(obstacles.size()); }
    int totalObstacleVertices() const {
        int v = 0;
        for (const auto& poly : obstacles) v += poly.size();
        return v;
    }
};

/**
 * @brief Generador determinista y reproducible de escenarios urbanos con obstáculos y clientes.
 * Emplea el motor pseudoaleatorio Mersenne Twister de 64 bits (std::mt19937_64).
 */
class DatasetGenerator {
public:
    /**
     * @brief Genera un escenario sintético acorde a los límites de la categoría especificada.
     * @param category Categoría de la instancia (SMALL, MEDIUM, LARGE, EXTREME).
     * @param seed Semilla determinista fijada para garantizar 100% de reproducibilidad.
     * @param gridWidth Dimensión del mapa en el eje X (ej. 1000.0).
     * @param gridHeight Dimensión del mapa en el eje Y (ej. 1000.0).
     */
    static Scenario generate(
        DatasetCategory category,
        uint64_t seed = 42ULL,
        double gridWidth = 1000.0,
        double gridHeight = 1000.0);

    /**
     * @brief Genera un escenario adversarial con trampas cóncavas (edificios en forma de U / C),
     * desafiando la heurística euclidiana directa (Sec. 5.2).
     */
    static Scenario generateAdversarialConcaveTrap(
        uint64_t seed = 1337ULL,
        double gridWidth = 1000.0,
        double gridHeight = 1000.0);

    /**
     * @brief Exporta un escenario a formato de texto estructurado para intercambio y persistencia.
     */
    static bool saveToFile(const Scenario& scenario, const std::string& filepath);

    /**
     * @brief Carga un escenario desde un archivo de texto estructurado.
     */
    static bool loadFromFile(Scenario& scenario, const std::string& filepath);
};

} // namespace geodrone

#endif // GEODRONE_DATASETGENERATOR_HPP
