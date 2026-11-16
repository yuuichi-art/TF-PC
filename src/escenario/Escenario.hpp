#ifndef GEODRONE_ESCENARIO_HPP
#define GEODRONE_ESCENARIO_HPP

#include "../geometria/Punto.hpp"
#include "../geometria/Poligono.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Categorías de evaluación. ADVERSARIAL agrupa los casos hechos a mano de data/adversarial/.
 */
enum class Categoria {
    SMALL,
    MEDIUM,
    LARGE,
    EXTREME,
    ADVERSARIAL
};

/**
 * @brief Límites competitivos de una categoría (confirmados con mediciones, ver docs/decisiones.md §7).
 */
struct LimitesCategoria {
    double limiteTiempoSegundos;
    double limiteMemoriaMB;
};

inline LimitesCategoria limitesDe(Categoria categoria) {
    switch (categoria) {
        case Categoria::SMALL:       return {1.0, 256.0};
        case Categoria::MEDIUM:      return {2.0, 256.0};
        case Categoria::LARGE:       return {3.0, 512.0};
        case Categoria::EXTREME:     return {5.0, 512.0};
        case Categoria::ADVERSARIAL: return {1.0, 256.0}; // Casos pequeños: mismos límites que Small
    }
    return {1.0, 256.0};
}

// Nombre de la categoría tal como se escribe en los archivos de escenario.
inline std::string nombreCategoria(Categoria categoria) {
    switch (categoria) {
        case Categoria::SMALL:       return "SMALL";
        case Categoria::MEDIUM:      return "MEDIUM";
        case Categoria::LARGE:       return "LARGE";
        case Categoria::EXTREME:     return "EXTREME";
        case Categoria::ADVERSARIAL: return "ADVERSARIAL";
    }
    return "SMALL";
}

// Convierte el texto del archivo en categoría. Devuelve false si el nombre no existe.
inline bool interpretarCategoria(const std::string& texto, Categoria& categoria) {
    const Categoria todas[] = {
        Categoria::SMALL, Categoria::MEDIUM, Categoria::LARGE,
        Categoria::EXTREME, Categoria::ADVERSARIAL
    };
    for (Categoria candidata : todas) {
        if (nombreCategoria(candidata) == texto) {
            categoria = candidata;
            return true;
        }
    }
    return false;
}

/**
 * @brief Escenario de misión: todo lo que contiene un archivo de escenario (docs/decisiones.md §6).
 * Nombre, categoría y semilla permiten reconstruir el caso con el generador.
 */
struct Escenario {
    std::string nombre;                  // Sin espacios: se guarda como una sola palabra
    Categoria categoria = Categoria::SMALL;
    uint64_t semilla = 0;              // 0 en los casos hechos a mano
    double anchoArea = 1000.0;         // Rectángulo que contiene el escenario
    double altoArea = 1000.0;
    double bateria = 0.0;              // Autonomía en unidades de distancia
    Punto base;
    std::vector<Punto> estaciones;
    std::vector<Punto> clientes;
    std::vector<Poligono> obstaculos;

    int numEstaciones() const { return static_cast<int>(estaciones.size()); }
    int numClientes() const { return static_cast<int>(clientes.size()); }
    int numObstaculos() const { return static_cast<int>(obstaculos.size()); }

    int totalVerticesObstaculos() const {
        int total = 0;
        for (const auto& poligono : obstaculos) total += poligono.numVertices();
        return total;
    }
};

} // namespace geodrone

#endif // GEODRONE_ESCENARIO_HPP
