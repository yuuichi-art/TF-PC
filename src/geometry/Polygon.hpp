#ifndef GEODRONE_POLYGON_HPP
#define GEODRONE_POLYGON_HPP

#include "Point.hpp"
#include "Segment.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <iostream>

namespace geodrone {

/**
 * @brief Polígono simple bidimensional que modela edificios, no-fly zones y áreas de exclusión.
 * Los vértices están ordenados secuencialmente alrededor del perímetro.
 */
struct Polygon {
    int id;
    std::string name;
    std::vector<Point> vertices;

    Polygon() : id(-1), name(""), vertices() {}
    Polygon(int id_, const std::string& name_ = "") : id(id_), name(name_), vertices() {}
    Polygon(int id_, const std::vector<Point>& vertices_, const std::string& name_ = "")
        : id(id_), name(name_), vertices(vertices_) {}

    // Número de vértices del polígono
    int size() const {
        return static_cast<int>(vertices.size());
    }

    bool empty() const {
        return vertices.empty();
    }

    // Acceso indexado a los vértices con manejo cíclico opcional
    const Point& operator[](int i) const {
        return vertices[i];
    }

    Point& operator[](int i) {
        return vertices[i];
    }

    const Point& at(int i) const {
        return vertices.at(i);
    }

    void addVertex(const Point& pt) {
        vertices.push_back(pt);
    }

    // Genera la lista de segmentos perimetrales del polígono (aristas)
    std::vector<Segment> getEdges() const {
        std::vector<Segment> edges;
        int n = size();
        if (n < 2) return edges;
        edges.reserve(n);
        for (int i = 0; i < n; ++i) {
            int next = (i + 1) % n;
            edges.emplace_back(vertices[i], vertices[next], i);
        }
        return edges;
    }

    // Perímetro total del polígono
    double perimeter() const {
        double perim = 0.0;
        int n = size();
        if (n < 2) return 0.0;
        for (int i = 0; i < n; ++i) {
            perim += vertices[i].dist(vertices[(i + 1) % n]);
        }
        return perim;
    }

    // Área con signo (fórmula de la lazada / Shoelace). Positiva si antihorario, negativa si horario.
    double signedArea() const {
        double area2 = 0.0;
        int n = size();
        if (n < 3) return 0.0;
        for (int i = 0; i < n; ++i) {
            int next = (i + 1) % n;
            area2 += vertices[i].cross(vertices[next]);
        }
        return area2 * 0.5;
    }

    // Área absoluta del polígono
    double area() const {
        return std::abs(signedArea());
    }

    // Determina si los vértices están listados en sentido horario
    bool isClockwise() const {
        return signedArea() < -EPS;
    }

    friend std::ostream& operator<<(std::ostream& os, const Polygon& poly) {
        os << "Polygon[id=" << poly.id;
        if (!poly.name.empty()) os << ", name=" << poly.name;
        os << ", vertices=" << poly.size() << "]";
        return os;
    }
};

} // namespace geodrone

#endif // GEODRONE_POLYGON_HPP
