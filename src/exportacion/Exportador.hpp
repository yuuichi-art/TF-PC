#ifndef GEODRONE_EXPORTADOR_HPP
#define GEODRONE_EXPORTADOR_HPP

#include "../juez/Juez.hpp"
#include "../escenario/Escenario.hpp"
#include "../solvers/Ruta.hpp"

#include <ostream>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Exportación de resultados (RF15) con los formatos de docs/decisiones.md §6.
 */
namespace Exportador {

    /**
     * @brief Escribe la ruta: resumen, clientes excluidos con su motivo, orden de nodos clave y
     * trayectoria completa con la etiqueta de cada punto. O(longitud de la trayectoria).
     */
    void escribirRuta(const Ruta& ruta, std::ostream& salida);

    /**
     * @brief Guarda la ruta en un archivo, creando las carpetas que falten.
     * Lanza std::runtime_error si no se puede escribir.
     */
    void guardarRuta(const Ruta& ruta, const std::string& nombreArchivo);

    /**
     * @brief Resultados del juez, una fila por escenario y algoritmo, con las columnas
     * escenario,categoria,clientes,algoritmo,veredicto,costo,recargas,excluidos,tiempo_ms,
     * memoria_mb,gap_mejor,gap_cota. Los gaps quedan vacíos si el veredicto no es OK.
     */
    void escribirResultadosCsv(const std::vector<FilaJuez>& filas, std::ostream& salida);
    void guardarResultadosCsv(const std::vector<FilaJuez>& filas, const std::string& nombreArchivo);

    /**
     * @brief Ranking por categoría: categoria,posicion,algoritmo,casos_ok,casos,gap_medio,tiempo_medio_ms.
     */
    void escribirRankingCsv(const std::vector<FilaRanking>& ranking, std::ostream& salida);
    void guardarRankingCsv(const std::vector<FilaRanking>& ranking, const std::string& nombreArchivo);

    /**
     * @brief Dibujo del escenario y la ruta en SVG, que se abre en cualquier navegador: obstáculos,
     * trayectoria, base, estaciones y clientes atendidos o excluidos, con título y leyenda.
     * Es solo visualización (opcional según la guía): la validación oficial es la del juez.
     * El eje y se invierte, porque en SVG crece hacia abajo. O(puntos + vértices de obstáculos).
     */
    void escribirSvg(const Escenario& escenario, const Ruta& ruta, std::ostream& salida);
    void guardarSvg(const Escenario& escenario, const Ruta& ruta, const std::string& nombreArchivo);

} // namespace Exportador

} // namespace geodrone

#endif // GEODRONE_EXPORTADOR_HPP
