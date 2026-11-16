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
    // Margen sobre la autonomía mínima: con 1,2 todos los clientes se pueden atender y aun
    // así hacen falta recargas, que es lo que se quiere poner a prueba.
    static constexpr double BATTERY_MARGIN = 1.2;

    static CategorySpec specFor(DatasetCategory category);

    // Los mismos rangos con cada cantidad fija en su máximo: el peor caso de la categoría.
    static CategorySpec worstCaseSpecFor(DatasetCategory category);

    /**
     * @brief Genera un escenario completo de la categoría dada (SMALL, MEDIUM, LARGE o EXTREME):
     * la geometría de generateLayout más la batería de batteryFor.
     * El nombre es "<categoria>_<semilla>", por ejemplo "small_3".
     * Lanza std::invalid_argument con ADVERSARIAL: esos casos se escriben a mano.
     */
    static Scenario generate(DatasetCategory category, uint64_t seed,
                             double width = 1000.0, double height = 1000.0);

    /**
     * @brief Solo la geometría: obstáculos, base, estaciones y clientes, con batería 0 (sin
     * calcular). Es rápida porque no construye el grafo de misión; la usan las pruebas y la
     * medición del peor caso (con todos los rangos en su máximo).
     */
    static Scenario generateLayout(const CategorySpec& spec, DatasetCategory category, uint64_t seed,
                                   double width = 1000.0, double height = 1000.0);

    /**
     * @brief Batería del escenario: la autonomía mínima (búsqueda binaria, RF10) por
     * BATTERY_MARGIN, redondeada hacia arriba a un entero. Al menos 1.
     * Construye el grafo de misión, así que cuesta lo mismo que prepararlo para resolver.
     */
    static double batteryFor(const Scenario& layout);

    /**
     * @brief Genera las semillas 1..seedsPerCategory de cada categoría y las guarda en folder
     * como "<categoria>_<semilla>.txt". Con includeWorstCase agrega "extreme_max.txt": un Extreme
     * con todos los rangos en su máximo (semilla 1), para comprobar que el peor caso cabe en el
     * límite. Devuelve las rutas creadas, de menor a mayor categoría.
     */
    static std::vector<std::string> generateBenchmarkSet(const std::string& folder, int seedsPerCategory = 5,
                                                         bool includeWorstCase = true);
};

} // namespace geodrone

#endif // GEODRONE_DATASETGENERATOR_HPP
