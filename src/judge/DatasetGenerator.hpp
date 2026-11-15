#ifndef GEODRONE_DATASETGENERATOR_HPP
#define GEODRONE_DATASETGENERATOR_HPP

#include "../scenario/Scenario.hpp"
#include <cstdint>

namespace geodrone {

/**
 * @brief Generador determinista y reproducible de escenarios urbanos con obstáculos y clientes.
 * Emplea el motor pseudoaleatorio Mersenne Twister de 64 bits (std::mt19937_64).
 * La lectura y escritura de escenarios está en scenario/ScenarioIO.
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
     * desafiando la heurística euclidiana directa.
     */
    static Scenario generateAdversarialConcaveTrap(
        uint64_t seed = 1337ULL,
        double gridWidth = 1000.0,
        double gridHeight = 1000.0);
};

} // namespace geodrone

#endif // GEODRONE_DATASETGENERATOR_HPP
