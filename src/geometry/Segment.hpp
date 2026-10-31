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

    // Comprueba si un punto P pertenece al segmento cerrado AB (incluyendo extremos)
    // Asume que la colinealidad puede ser verificada con EPS
    bool containsPoint(const Point& p) const {
        // Colinealidad: producto cruz (B - A) x (P - A) == 0
        Point ab = b - a;
        Point ap = p - a;
        if (std::abs(ab.cross(ap)) > EPS) {
            return false;
        }
        // Caja delimitadora (Bounding Box)
        double minX = std::min(a.x, b.x) - EPS;
        double maxX = std::max(a.x, b.x) + EPS;
        double minY = std::min(a.y, b.y) - EPS;
        double maxY = std::max(a.y, b.y) + EPS;

        return (p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY);
    }

    friend std::ostream& operator<<(std::ostream& os, const Segment& s) {
        os << "Segment[" << s.a << " -> " << s.b << "]";
        return os;
    }
};

} // namespace geodrone

#endif // GEODRONE_SEGMENT_HPP
