#ifndef GEODRONE_EXPORTER_HPP
#define GEODRONE_EXPORTER_HPP

#include "../solvers/Route.hpp"

#include <ostream>
#include <string>

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

} // namespace Exporter

} // namespace geodrone

#endif // GEODRONE_EXPORTER_HPP
