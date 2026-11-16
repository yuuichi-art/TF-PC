#ifndef GEODRONE_SEGMENTO_HPP
#define GEODRONE_SEGMENTO_HPP

#include "Punto.hpp"

namespace geodrone {

/**
 * @brief Segmento de trayectoria de vuelo delimitado por dos puntos extremos A y B.
 */
struct Segment {
    Point a;
    Point b;

    Segment() = default;
    Segment(const Point& a_, const Point& b_) : a(a_), b(b_) {}

    // Longitud euclidiana del segmento
    double length() const {
        return a.dist(b);
    }

    // Punto medio del segmento (utilizado para validar si el segmento penetra obstáculos)
    Point midpoint() const {
        return Point((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
    }

    // Para saber si un punto está sobre el segmento se usa GeometryUtils::onSegment.
};

} // namespace geodrone

#endif // GEODRONE_SEGMENTO_HPP
