#ifndef GEODRONE_JUDGE_HPP
#define GEODRONE_JUDGE_HPP

#include "../scenario/Scenario.hpp"
#include "../solvers/MissionSolver.hpp"
#include "../solvers/Route.hpp"

#include <ostream>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Resultado de verificar una ruta: válida, o el primer problema encontrado.
 */
struct VerificationResult {
    bool valid = true;
    std::string problem;
};

/**
 * @brief Una fila del CSV de resultados: un escenario resuelto con un algoritmo.
 */
struct JudgeRow {
    std::string scenario;
    DatasetCategory category = DatasetCategory::SMALL;
    int clients = 0;                // Clientes del escenario, incluidos los excluidos
    std::string algorithm;
    std::string verdict;            // OK, WA, TLE o MLE
    std::string problem;            // Motivo del WA (se muestra en consola, no va al CSV)
    double cost = 0.0;
    int recharges = 0;
    int excluded = 0;
    double timeMs = 0.0;            // Preparación + resolución
    double memoryMB = 0.0;          // Memoria pico del proceso al terminar el caso
    double gapBest = 0.0;           // % sobre la mejor ruta OK del caso; solo con veredicto OK
    double gapBound = 0.0;          // % sobre la cota del árbol de expansión mínima; solo con OK
};

/**
 * @brief Una fila del ranking de una categoría.
 */
struct RankingRow {
    DatasetCategory category = DatasetCategory::SMALL;
    int position = 0;
    std::string algorithm;
    int okCases = 0;
    int cases = 0;
    double meanGap = 0.0;           // Promedio de gapBest en los casos OK
    double meanTimeMs = 0.0;        // Promedio de tiempo en los casos OK
};

/**
 * @brief Juez competitivo y benchmark (RF14), según docs/decisiones.md §5.
 */
namespace Judge {

    /**
     * @brief Verifica una ruta con código independiente de los solvers: solo usa la ruta, el
     * escenario y la geometría. Comprueba que
     * - empieza y termina en la base, y cada punto etiquetado está donde dice el escenario;
     * - ningún tramo cruza un obstáculo;
     * - la batería simulada tramo a tramo nunca baja de 0 (se llena en la base y las estaciones);
     * - cada cliente se visita una vez o está excluido, y cada exclusión es real según el grafo
     *   de misión (separado por obstáculos o con 2·dRecarga > B);
     * - el costo y las recargas informados coinciden con los recalculados.
     * Complejidad: O(P · M) con P puntos de la trayectoria y M aristas de obstáculos.
     */
    VerificationResult verifyRoute(const Scenario& scenario, const MissionSolver& solver, const Route& route);

    /**
     * @brief Veredicto: WA si la ruta no es válida; si es válida, TLE si excede el tiempo,
     * MLE si excede la memoria y OK en otro caso.
     */
    std::string verdictFor(bool valid, double timeMs, double memoryMB, const CategoryLimits& limits);

    // Archivos .txt de una carpeta, ordenados por nombre. Vacío si la carpeta no existe.
    std::vector<std::string> findScenarioFiles(const std::string& folder);

    /**
     * @brief Ejecuta todos los algoritmos sobre todos los escenarios, de menor a mayor tamaño,
     * para que la memoria pico de cada caso sea la de ese caso. El exacto solo corre en Small y
     * en los adversariales. Guarda cada ruta en routesFolder y escribe el avance en log.
     */
    std::vector<JudgeRow> run(const std::vector<std::string>& scenarioFiles, const std::string& routesFolder,
                              std::ostream& log);

    /**
     * @brief Ranking por categoría: más casos OK, luego menor gap medio, luego menor tiempo medio.
     */
    std::vector<RankingRow> rank(const std::vector<JudgeRow>& rows);

} // namespace Judge

} // namespace geodrone

#endif // GEODRONE_JUDGE_HPP
