#ifndef GEODRONE_DATASETGENERATOR_HPP
#define GEODRONE_DATASETGENERATOR_HPP

#include "Scenario.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Rangos de tamaño de una categoría (confirmados con mediciones, ver docs/decisiones.md §7).
 * El generador elige cada cantidad al azar dentro de su rango.
 */
struct CategorySpec {
    int minClients;
    int maxClients;
    int minStations;
    int maxStations;
    int minObstacles;
    int maxObstacles;
    int maxPointsPerObstacle;   // Puntos aleatorios por obstáculo; su envolvente puede tener menos vértices
};

/**
 * @brief Generador determinista y reproducible de escenarios con obstáculos, estaciones y clientes.
 *
 * - Usa std::mt19937_64 con la semilla recibida: la misma semilla da el mismo escenario.
 * - Coordenadas enteras dentro del área (decisión 1).
 * - Obstáculos convexos sin solape (decisión 2): el área se divide en celdas y cada
 *   obstáculo es la envolvente convexa de puntos aleatorios dentro de una celda distinta.
 * - Base, estaciones y clientes en puntos libres: fuera de todo obstáculo y sin repetir.
 *
 * Complejidad: O(M·k log k) para M obstáculos de k puntos, más O((N + S)·V) para colocar
 * N clientes y S estaciones comprobando los V vértices de obstáculos.
 */
class DatasetGenerator {
public:
    // Batería provisional: alcanza para cualquier tramo directo dentro del área de 1000 x 1000.
    // Se reemplazará por la autonomía mínima calculada con búsqueda binaria.
    static constexpr double PROVISIONAL_BATTERY = 2000.0;

    static CategorySpec specFor(DatasetCategory category);

    /**
     * @brief Genera un escenario de la categoría dada (SMALL, MEDIUM, LARGE o EXTREME).
     * El nombre es "<categoria>_<semilla>", por ejemplo "small_3".
     * Lanza std::invalid_argument con ADVERSARIAL: esos casos se escriben a mano.
     */
    static Scenario generate(DatasetCategory category, uint64_t seed,
                             double width = 1000.0, double height = 1000.0);

    /**
     * @brief Igual que generate(category, seed), pero con rangos de tamaño propios.
     * Sirve para medir el peor caso de una categoría (todos los rangos en su máximo).
     */
    static Scenario generate(const CategorySpec& spec, DatasetCategory category, uint64_t seed,
                             double width = 1000.0, double height = 1000.0);

    /**
     * @brief Genera las semillas 1..seedsPerCategory de cada categoría y las guarda en folder
     * como "<categoria>_<semilla>.txt". Devuelve las rutas creadas, de menor a mayor categoría.
     */
    static std::vector<std::string> generateBenchmarkSet(const std::string& folder, int seedsPerCategory = 5);
};

} // namespace geodrone

#endif // GEODRONE_DATASETGENERATOR_HPP
