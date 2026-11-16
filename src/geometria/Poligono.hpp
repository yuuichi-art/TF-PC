#ifndef GEODRONE_POLIGONO_HPP
#define GEODRONE_POLIGONO_HPP

#include "Punto.hpp"
#include "Segmento.hpp"
#include <cmath>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Polígono simple bidimensional que modela edificios, no-fly zones y áreas de exclusión.
 * Los vértices están ordenados secuencialmente alrededor del perímetro.
 */
struct Poligono {
    int id;
    std::string nombre;
    std::vector<Punto> vertices;

    Poligono() : id(-1), nombre(""), vertices() {}
    Poligono(int id_, const std::string& nombre_ = "") : id(id_), nombre(nombre_), vertices() {}
    Poligono(int id_, const std::vector<Punto>& vertices_, const std::string& nombre_ = "")
        : id(id_), nombre(nombre_), vertices(vertices_) {}

    // Número de vértices del polígono
    int numVertices() const {
        return static_cast<int>(vertices.size());
    }

    // Acceso indexado a los vértices (sin comprobar el rango)
    const Punto& operator[](int i) const {
        return vertices[i];
    }

    Punto& operator[](int i) {
        return vertices[i];
    }

    void agregarVertice(const Punto& punto) {
        vertices.push_back(punto);
    }

    // Lados del polígono como segmentos (el último une el último vértice con el primero)
    std::vector<Segmento> lados() const {
        std::vector<Segmento> resultado;
        int n = numVertices();
        if (n < 2) return resultado;
        resultado.reserve(n);
        for (int i = 0; i < n; ++i) {
            int siguiente = (i + 1) % n;
            resultado.emplace_back(vertices[i], vertices[siguiente]);
        }
        return resultado;
    }

    // Perímetro total del polígono
    double perimetro() const {
        double suma = 0.0;
        int n = numVertices();
        if (n < 2) return 0.0;
        for (int i = 0; i < n; ++i) {
            suma += vertices[i].distancia(vertices[(i + 1) % n]);
        }
        return suma;
    }

    // Área con signo (fórmula de la lazada / Shoelace). Positiva si antihorario, negativa si horario.
    double areaConSigno() const {
        double area2 = 0.0;
        int n = numVertices();
        if (n < 3) return 0.0;
        for (int i = 0; i < n; ++i) {
            int siguiente = (i + 1) % n;
            area2 += vertices[i].cruz(vertices[siguiente]);
        }
        return area2 * 0.5;
    }

    // Área absoluta del polígono
    double area() const {
        return std::abs(areaConSigno());
    }
};

} // namespace geodrone

#endif // GEODRONE_POLIGONO_HPP
