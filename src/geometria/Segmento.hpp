#ifndef GEODRONE_SEGMENTO_HPP
#define GEODRONE_SEGMENTO_HPP

#include "Punto.hpp"

namespace geodrone {

/**
 * @brief Segmento de trayectoria de vuelo delimitado por dos puntos extremos A y B.
 */
struct Segmento {
    Punto a;
    Punto b;

    Segmento() = default;
    Segmento(const Punto& a_, const Punto& b_) : a(a_), b(b_) {}

    // Longitud euclidiana del segmento
    double longitud() const {
        return a.distancia(b);
    }

    // Punto medio del segmento (utilizado para validar si el segmento penetra obstáculos)
    Punto puntoMedio() const {
        return Punto((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
    }

    // Para saber si un punto está sobre el segmento se usa Geometria::estaEnSegmento.
};

} // namespace geodrone

#endif // GEODRONE_SEGMENTO_HPP
