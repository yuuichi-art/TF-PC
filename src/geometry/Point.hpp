#ifndef GEODRONE_POINT_HPP
#define GEODRONE_POINT_HPP

#include <cmath>
#include <limits>

namespace geodrone {

// Tolerancia numérica estándar para operaciones geométricas en punto flotante
inline constexpr double EPS = 1e-9;

// Distancia "infinita": no hay camino. Única definición para todo el proyecto.
inline constexpr double INF = std::numeric_limits<double>::infinity();

/**
 * @brief Punto o vector en el plano R^2. Las comparaciones usan la tolerancia EPS.
 */
struct Point {
    double x;
    double y;

    constexpr Point() : x(0.0), y(0.0) {}
    constexpr Point(double x_, double y_) : x(x_), y(y_) {}

    // Operadores vectoriales
    Point operator+(const Point& o) const {
        return Point(x + o.x, y + o.y);
    }

    Point operator-(const Point& o) const {
        return Point(x - o.x, y - o.y);
    }

    Point operator*(double k) const {
        return Point(x * k, y * k);
    }

    // Producto cruz 2D (determinante 2x2): x1 * y2 - y1 * x2
    // Positivo: giro antihorario (izquierda).
    // Negativo: giro horario (derecha).
    // Cero: colinealidad.
    double cross(const Point& o) const {
        return x * o.y - y * o.x;
    }

    // Distancia euclidiana usando std::hypot para evitar overflow / underflow numérico
    double dist(const Point& o) const {
        return std::hypot(x - o.x, y - o.y);
    }

    // Igualdad con tolerancia EPS en cada coordenada
    bool operator==(const Point& o) const {
        return std::abs(x - o.x) < EPS && std::abs(y - o.y) < EPS;
    }

    bool operator!=(const Point& o) const {
        return !(*this == o);
    }

    // Orden lexicográfico (primero x, luego y) con tolerancia EPS; lo usa la envolvente convexa
    bool operator<(const Point& o) const {
        if (std::abs(x - o.x) > EPS) {
            return x < o.x;
        }
        return y < o.y - EPS;
    }
};

} // namespace geodrone

#endif // GEODRONE_POINT_HPP
