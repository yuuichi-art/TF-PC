#include "Geometria.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace geodrone {
namespace Geometria {

int orientacion(const Punto& a, const Punto& b, const Punto& c) {
    double producto = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (producto > EPS) return 1;    // Antihorario / izquierda
    if (producto < -EPS) return -1;  // Horario / derecha
    return 0;                        // Colineal
}

bool estaEnSegmento(const Punto& p, const Punto& a, const Punto& b) {
    // Verificar colinealidad con tolerancia EPS
    if (std::abs((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) > EPS) {
        return false;
    }
    // Verificar si p reside dentro del bounding box con EPS
    double minX = std::min(a.x, b.x) - EPS;
    double maxX = std::max(a.x, b.x) + EPS;
    double minY = std::min(a.y, b.y) - EPS;
    double maxY = std::max(a.y, b.y) + EPS;

    return (p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY);
}

bool estaEnSegmento(const Punto& p, const Segmento& s) {
    return estaEnSegmento(p, s.a, s.b);
}

bool segmentosSeIntersecan(const Segmento& s1, const Segmento& s2, bool soloPropia) {
    const Punto& p1 = s1.a;
    const Punto& q1 = s1.b;
    const Punto& p2 = s2.a;
    const Punto& q2 = s2.b;

    int o1 = orientacion(p1, q1, p2);
    int o2 = orientacion(p1, q1, q2);
    int o3 = orientacion(p2, q2, p1);
    int o4 = orientacion(p2, q2, q1);

    // Intersección propia estricta (en cruz, sin que ningún extremo toque el otro segmento)
    bool propia = (o1 * o2 < 0) && (o3 * o4 < 0);
    if (soloPropia) {
        return propia;
    }
    if (propia) {
        return true;
    }

    // Casos especiales: puntos colineales donde un extremo reside dentro del otro segmento
    if (o1 == 0 && estaEnSegmento(p2, p1, q1)) return true;
    if (o2 == 0 && estaEnSegmento(q2, p1, q1)) return true;
    if (o3 == 0 && estaEnSegmento(p1, p2, q2)) return true;
    if (o4 == 0 && estaEnSegmento(q1, p2, q2)) return true;

    return false;
}

bool interseccionPropia(const Segmento& s1, const Segmento& s2) {
    return segmentosSeIntersecan(s1, s2, true);
}

UbicacionPunto ubicacionEnPoligono(const Punto& punto, const Poligono& poligono) {
    int n = poligono.numVertices();
    if (n < 3) return UbicacionPunto::FUERA;

    // 1. Validar primero si el punto reside sobre alguna arista perimetral o vértice
    for (int i = 0; i < n; ++i) {
        int siguiente = (i + 1) % n;
        if (estaEnSegmento(punto, poligono[i], poligono[siguiente])) {
            return UbicacionPunto::BORDE;
        }
    }

    // 2. Algoritmo de Ray Casting (Teorema de la curva de Jordan)
    // Se proyecta un rayo horizontal hacia x -> +inf
    int cruces = 0;
    for (int i = 0; i < n; ++i) {
        Punto v1 = poligono[i];
        Punto v2 = poligono[(i + 1) % n];

        // Normalizar para que v1 tenga menor o igual coordenada Y
        if (v1.y > v2.y) {
            std::swap(v1, v2);
        }

        // Evaluar si el rayo horizontal en punto.y cruza el semi-intervalo [v1.y, v2.y)
        // La condición semi-abierta previene contar dos veces cuando el rayo cruza un vértice común.
        if (punto.y >= v1.y - EPS && punto.y < v2.y - EPS) {
            double dy = v2.y - v1.y;
            if (std::abs(dy) > EPS) {
                double xCruce = v1.x + (punto.y - v1.y) * (v2.x - v1.x) / dy;
                if (xCruce > punto.x + EPS) {
                    cruces++;
                }
            }
        }
    }

    return (cruces % 2 == 1) ? UbicacionPunto::DENTRO : UbicacionPunto::FUERA;
}

bool estaEstrictamenteDentro(const Punto& punto, const Poligono& poligono) {
    return ubicacionEnPoligono(punto, poligono) == UbicacionPunto::DENTRO;
}

bool segmentoCruzaInterior(const Segmento& segmento, const Poligono& poligono) {
    int n = poligono.numVertices();
    if (n < 3) return false;

    // Caso de punto degenerado
    if (segmento.a == segmento.b) {
        return estaEstrictamenteDentro(segmento.a, poligono);
    }

    // 1. Si el segmento interseca propiamente (en cruz) cualquier lado del polígono,
    // forzosamente penetra o atraviesa el interior.
    // Los lados se arman en la pila en vez de pedir poligono.lados(): esta función se llama
    // millones de veces al construir el grafo y crear un vector en cada llamada era el costo dominante.
    for (int i = 0; i < n; ++i) {
        Segmento arista(poligono[i], poligono[(i + 1) % n]);
        if (interseccionPropia(segmento, arista)) {
            return true;
        }
    }

    // 2. Si no hay intersección propia en cruz, podría ser una cuerda interna
    // (ej. conectando dos vértices del polígono a través del interior) o un segmento
    // que toca un vértice tangencialmente.
    // Caso rápido (el más común): si ningún vértice está sobre el segmento, el segmento no
    // cruza el borde en ningún punto interior, así que queda entero dentro o entero fuera:
    // basta revisar su punto medio.
    bool tocaVertice = false;
    for (int i = 0; i < n && !tocaVertice; ++i) {
        tocaVertice = estaEnSegmento(poligono[i], segmento);
    }
    if (!tocaVertice) {
        return estaEstrictamenteDentro(segmento.puntoMedio(), poligono);
    }

    // Caso general: el segmento pasa por vértices. Recolectamos las proyecciones t en [0, 1]
    // de esos vértices y revisamos el punto medio de cada sub-intervalo.
    std::vector<double> muestrasT = {0.0, 1.0};

    Punto direccion = segmento.b - segmento.a;
    double longitudCuadrada = direccion.x * direccion.x + direccion.y * direccion.y;

    if (longitudCuadrada > EPS) {
        for (int i = 0; i < n; ++i) {
            Punto p = poligono[i];
            if (estaEnSegmento(p, segmento)) {
                double t = ((p.x - segmento.a.x) * direccion.x + (p.y - segmento.a.y) * direccion.y) / longitudCuadrada;
                if (t > EPS && t < 1.0 - EPS) {
                    muestrasT.push_back(t);
                }
            }
        }
    }

    std::sort(muestrasT.begin(), muestrasT.end());
    muestrasT.erase(std::unique(muestrasT.begin(), muestrasT.end(), [](double a, double b) {
        return std::abs(a - b) < EPS;
    }), muestrasT.end());

    // Evaluamos el punto medio de cada sub-intervalo libre
    for (size_t i = 0; i + 1 < muestrasT.size(); ++i) {
        double tMedio = (muestrasT[i] + muestrasT[i + 1]) * 0.5;
        Punto puntoPrueba = segmento.a + direccion * tMedio;
        if (estaEstrictamenteDentro(puntoPrueba, poligono)) {
            return true;
        }
    }

    return false;
}

bool tramoLibre(const Segmento& segmento, const std::vector<Poligono>& obstaculos) {
    for (const auto& poligono : obstaculos) {
        if (segmentoCruzaInterior(segmento, poligono)) {
            return false;
        }
    }
    return true;
}

} // namespace Geometria
} // namespace geodrone
