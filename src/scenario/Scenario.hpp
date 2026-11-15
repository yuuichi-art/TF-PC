#ifndef GEODRONE_SCENARIO_HPP
#define GEODRONE_SCENARIO_HPP

#include "../geometry/Point.hpp"
#include "../geometry/Polygon.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Categorías de evaluación. ADVERSARIAL agrupa los casos hechos a mano de data/adversarial/.
 */
enum class DatasetCategory {
    SMALL,
    MEDIUM,
    LARGE,
    EXTREME,
    ADVERSARIAL
};

/**
 * @brief Límites competitivos de una categoría (confirmados con mediciones, ver docs/decisiones.md §7).
 */
struct CategoryLimits {
    double timeLimitSeconds;
    double memoryLimitMB;
};

inline CategoryLimits limitsFor(DatasetCategory category) {
    switch (category) {
        case DatasetCategory::SMALL:       return {1.0, 256.0};
        case DatasetCategory::MEDIUM:      return {2.0, 256.0};
        case DatasetCategory::LARGE:       return {3.0, 512.0};
        case DatasetCategory::EXTREME:     return {5.0, 512.0};
        case DatasetCategory::ADVERSARIAL: return {1.0, 256.0}; // Casos pequeños: mismos límites que Small
    }
    return {1.0, 256.0};
}

// Nombre de la categoría tal como se escribe en los archivos de escenario.
inline std::string categoryName(DatasetCategory category) {
    switch (category) {
        case DatasetCategory::SMALL:       return "SMALL";
        case DatasetCategory::MEDIUM:      return "MEDIUM";
        case DatasetCategory::LARGE:       return "LARGE";
        case DatasetCategory::EXTREME:     return "EXTREME";
        case DatasetCategory::ADVERSARIAL: return "ADVERSARIAL";
    }
    return "SMALL";
}

// Convierte el texto del archivo en categoría. Devuelve false si el nombre no existe.
inline bool parseCategory(const std::string& text, DatasetCategory& category) {
    const DatasetCategory all[] = {
        DatasetCategory::SMALL, DatasetCategory::MEDIUM, DatasetCategory::LARGE,
        DatasetCategory::EXTREME, DatasetCategory::ADVERSARIAL
    };
    for (DatasetCategory candidate : all) {
        if (categoryName(candidate) == text) {
            category = candidate;
            return true;
        }
    }
    return false;
}

/**
 * @brief Escenario de misión: todo lo que contiene un archivo de escenario (docs/decisiones.md §6).
 * Nombre, categoría y semilla permiten reconstruir el caso con el generador.
 */
struct Scenario {
    std::string name;                  // Sin espacios: se guarda como una sola palabra
    DatasetCategory category = DatasetCategory::SMALL;
    uint64_t seed = 0;                 // 0 en los casos hechos a mano
    double areaWidth = 1000.0;         // Rectángulo que contiene el escenario
    double areaHeight = 1000.0;
    double battery = 0.0;              // Autonomía en unidades de distancia
    Point depot;
    std::vector<Point> stations;
    std::vector<Point> clients;
    std::vector<Polygon> obstacles;

    int numStations() const { return static_cast<int>(stations.size()); }
    int numClients() const { return static_cast<int>(clients.size()); }
    int numObstacles() const { return static_cast<int>(obstacles.size()); }

    int totalObstacleVertices() const {
        int total = 0;
        for (const auto& poly : obstacles) total += poly.size();
        return total;
    }
};

} // namespace geodrone

#endif // GEODRONE_SCENARIO_HPP
