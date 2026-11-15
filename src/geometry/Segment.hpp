#ifndef GEODRONE_SEGMENT_HPP
#define GEODRONE_SEGMENT_HPP

#include "Point.hpp"
#include <algorithm>
#include <iostream>

namespace geodrone {

/**
 * @brief Segmento de trayectoria de vuelo delimitado por dos puntos extremos A y B.
 */
struct Segment {
    Point a;
    Point b;
    int id;

    Segment() : a(), b(), id(-1) {}
    Segment(const Point& a_, const Point& b_, int id_ = -1) : a(a_), b(b_), id(id_) {}

    // Longitud euclidiana del segmento
    double length() const {
        return a.dist(b);
    }

    // Punto medio del segmento (utilizado para validar si el segmento penetra obstáculos)
    Point midpoint() const {
        return Point((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
    }

    // Vector director del segmento (B - A)
    Point toVector() const {
        return b - a;
    }

    // Para saber si un punto está sobre el segmento se usa GeometryUtils::onSegment.

    friend std::ostream& operator<<(std::ostream& os, const Segment& s) {
        os << "Segment[" << s.a << " -> " << s.b << "]";
        return os;
    }
};

} // namespace geodrone

#endif // GEODRONE_SEGMENT_HPP
