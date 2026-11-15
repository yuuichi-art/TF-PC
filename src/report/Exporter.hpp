#ifndef GEODRONE_EXPORTER_HPP
#define GEODRONE_EXPORTER_HPP

#include "../judge/Judge.hpp"
#include "../solvers/Route.hpp"

#include <ostream>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Exportación de resultados (RF15) con los formatos de docs/decisiones.md §6.
 */
namespace Exporter {

    /**
     * @brief Escribe la ruta: resumen, clientes excluidos con su motivo, orden de nodos clave y
     * trayectoria completa con la etiqueta de cada punto. O(longitud de la trayectoria).
     */
    void writeRoute(const Route& route, std::ostream& out);

    /**
     * @brief Guarda la ruta en un archivo, creando las carpetas que falten.
     * Lanza std::runtime_error si no se puede escribir.
     */
    void saveRoute(const Route& route, const std::string& path);

    /**
     * @brief Resultados del juez, una fila por escenario y algoritmo, con las columnas
     * escenario,categoria,clientes,algoritmo,veredicto,costo,recargas,excluidos,tiempo_ms,
     * memoria_mb,gap_mejor,gap_cota. Los gaps quedan vacíos si el veredicto no es OK.
     */
    void writeResultsCsv(const std::vector<JudgeRow>& rows, std::ostream& out);
    void saveResultsCsv(const std::vector<JudgeRow>& rows, const std::string& path);

    /**
     * @brief Ranking por categoría: categoria,posicion,algoritmo,casos_ok,casos,gap_medio,tiempo_medio_ms.
     */
    void writeRankingCsv(const std::vector<RankingRow>& ranking, std::ostream& out);
    void saveRankingCsv(const std::vector<RankingRow>& ranking, const std::string& path);

} // namespace Exporter

} // namespace geodrone

#endif // GEODRONE_EXPORTER_HPP
