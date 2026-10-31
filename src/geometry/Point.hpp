#ifndef GEODRONE_POINT_HPP
#define GEODRONE_POINT_HPP

#include <cmath>
#include <iostream>
#include <iomanip>

namespace geodrone {

// Tolerancia numérica estándar para operaciones geométricas en punto flotante
inline constexpr double EPS = 1e-9;

/**
 * @brief Estructura que representa un punto o vector en el plano bidimensional R^2.
 * Optimizado para programación competitiva con baja sobrecarga de memoria y operaciones inline.
 */
struct Point {
    double x;
    double y;
    int id; // Identificador único opcional (-1 por defecto)

    constexpr Point() : x(0.0), y(0.0), id(-1) {}
    constexpr Point(double x_, double y_, int id_ = -1) : x(x_), y(y_), id(id_) {}

    // Operadores vectoriales
    Point operator+(const Point& o) const {
        return Point(x + o.x, y + o.y, id);
    }

    Point operator-(const Point& o) const {
        return Point(x - o.x, y - o.y, id);
    }

    Point operator*(double k) const {
        return Point(x * k, y * k, id);
    }

    Point operator/(double k) const {
        return Point(x / k, y / k, id);
    }

    // Producto punto (dot product): x1 * x2 + y1 * y2
    // Útil para proyecciones y verificar perpendicularidad.
    double dot(const Point& o) const {
        return x * o.x + y * o.y;
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

    // Distancia euclidiana al cuadrado (evita cálculo de raíz cuando solo se comparan distancias)
    double distSq(const Point& o) const {
        double dx = x - o.x;
        double dy = y - o.y;
        return dx * dx + dy * dy;
    }

    // Longitud / magnitud del vector
    double norm() const {
        return std::hypot(x, y);
    }

    // Comparadores de igualdad y orden lexicográfico con tolerancia épsilon (EPS)
    bool operator==(const Point& o) const {
        return std::abs(x - o.x) < EPS && std::abs(y - o.y) < EPS;
    }

    bool operator!=(const Point& o) const {
        return !(*this == o);
    }

    // Ordenamiento lexicográfico: primero por x, luego por y
    bool operator<(const Point& o) const {
        if (std::abs(x - o.x) > EPS) {
            return x < o.x;
        }
        return y < o.y - EPS;
    }

    bool operator>(const Point& o) const {
        return o < *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const Point& p) {
        os << "(" << std::fixed << std::setprecision(4) << p.x << ", " << p.y;
        if (p.id != -1) os << " [id=" << p.id << "]";
        os << ")";
        return os;
    }
};

} // namespace geodrone

#endif // GEODRONE_POINT_HPP
