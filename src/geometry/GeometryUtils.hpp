#ifndef GEODRONE_GEOMETRYUTILS_HPP
#define GEODRONE_GEOMETRYUTILS_HPP

#include "Point.hpp"
#include "Segment.hpp"
#include "Polygon.hpp"
#include <vector>

namespace geodrone {

/**
 * @brief Ubicación de un punto respecto a un polígono.
 */
enum class PointLocation {
    OUTSIDE = 0,    // Estrictamente exterior
    BOUNDARY = 1,   // Sobre el borde o vértice
    INSIDE = 2      // Estrictamente interior
};

namespace GeometryUtils {

    /**
     * @brief Calcula la orientación relativa 2D (CCW) de la terna (A, B, C).
     * Retorna +1 (antihorario / izquierda), -1 (horario / derecha) o 0 (colineal).
     */
    int ccw(const Point& a, const Point& b, const Point& c);

    /**
     * @brief Verifica si el punto P está sobre el segmento cerrado AB: colineal con A y B, y dentro
     * del rectángulo que forman (con tolerancia EPS).
     */
    bool onSegment(const Point& p, const Point& a, const Point& b);
    bool onSegment(const Point& p, const Segment& s);

    /**
     * @brief Determina si dos segmentos s1 y s2 se intersecan en el plano.
     * @param properOnly Si es true, solo reporta cruce estricto en el interior de ambos segmentos (sin tocar extremos).
     */
    bool segmentsIntersect(const Segment& s1, const Segment& s2, bool properOnly = false);

    /**
     * @brief Determina si dos segmentos tienen una intersección propia (en cruz, sin extremos).
     */
    bool properIntersection(const Segment& s1, const Segment& s2);

    /**
     * @brief Determina si un punto está dentro, en el borde o fuera de un polígono mediante Ray Casting (teorema de la curva de Jordan).
     * Complejidad: O(V) donde V es el número de vértices del polígono.
     */
    PointLocation pointInPolygon(const Point& pt, const Polygon& poly);

    /**
     * @brief true si el punto está estrictamente en el interior del polígono (no en el borde).
     */
    bool isPointStrictlyInside(const Point& pt, const Polygon& poly);

    /**
     * @brief Valida si una trayectoria entre u y v interseca o penetra el interior de un polígono de exclusión.
     * Requisito RF04: segmento(u, v) ∩ interior(poly) = ∅.
     * Complejidad, con n vértices del polígono: O(n) en el caso común (el tramo no pasa por
     * vértices); O(n^2) si pasa por vértices, porque se revisa un punto medio por cada tramo
     * entre ellos y cada revisión es O(n). Espacio O(1) en el caso común, O(n) en el otro.
     */
    bool segmentIntersectsPolygonInterior(const Segment& seg, const Polygon& poly);

    /**
     * @brief Valida si una trayectoria entre u y v es factible frente a una colección de obstáculos poligonales.
     * Complejidad: O(M) en total, con M el número de aristas de todos los obstáculos (caso común).
     */
    bool isFlightPathClear(const Segment& seg, const std::vector<Polygon>& obstacles);

} // namespace GeometryUtils

} // namespace geodrone

#endif // GEODRONE_GEOMETRYUTILS_HPP
