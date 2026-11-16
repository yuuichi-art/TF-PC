#ifndef GEODRONE_ARCHIVOESCENARIO_HPP
#define GEODRONE_ARCHIVOESCENARIO_HPP

#include "Escenario.hpp"

#include <fstream>
#include <istream>
#include <ostream>
#include <string>

namespace geodrone {

/**
 * @brief Lectura y escritura de escenarios en TXT con etiquetas en orden fijo (docs/decisiones.md §6):
 * NAME, CATEGORY, SEED, AREA, BATTERY, DEPOT, STATIONS, CLIENTS y OBSTACLES.
 *
 * Complejidad: O(T) en tiempo y espacio, donde T es la cantidad de valores del archivo.
 */
namespace ArchivoEscenario {

    /**
     * @brief Lee un escenario desde un flujo.
     * Lanza std::runtime_error con la línea y lo que se esperaba si el contenido no es válido.
     */
    Escenario leer(std::istream& entrada);

    /**
     * @brief Lee un escenario desde un archivo. El mensaje de error incluye la ruta del archivo.
     */
    Escenario cargar(const std::string& ruta);

    /**
     * @brief Escribe un escenario en el mismo formato que leer() acepta.
     * Lanza std::invalid_argument si el nombre está vacío o tiene espacios.
     */
    void escribir(const Escenario& escenario, std::ostream& salida);

    /**
     * @brief Guarda un escenario en un archivo, creando las carpetas que falten.
     * Lanza std::invalid_argument si el nombre no es válido y std::runtime_error si no se
     * puede escribir.
     */
    void guardar(const Escenario& escenario, const std::string& ruta);

    /**
     * @brief Crea el archivo para escribir, con las carpetas que falten. Lanza std::runtime_error
     * si no se puede. La usan también las exportaciones de rutas, CSV y SVG.
     */
    std::ofstream abrirParaEscribir(const std::string& ruta);

} // namespace ArchivoEscenario

} // namespace geodrone

#endif // GEODRONE_ARCHIVOESCENARIO_HPP
