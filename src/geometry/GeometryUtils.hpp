#ifndef GEODRONE_GEOMETRYUTILS_HPP
#define GEODRONE_GEOMETRYUTILS_HPP

#include "Point.hpp"
#include "Segment.hpp"
#include "Polygon.hpp"
#include <vector>

namespace geodrone {

/**
 * @brief Orientación relativa de una terna de puntos (A, B, C).
 */
enum class Orientation {
    CLOCKWISE = -1,         // Giro horario / derecha
    COLLINEAR = 0,          // Colineales
    COUNTER_CLOCKWISE = 1   // Giro antihorario / izquierda
};

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
     * @brief Retorna el enum Orientation correspondiente a la terna (A, B, C).
     */
    Orientation getOrientation(const Point& a, const Point& b, const Point& c);

    /**
     * @brief Verifica si el punto P reside sobre el segmento cerrado AB, asumiendo colinealidad previa o verificándola.
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
     * @brief Determina si un punto está dentro, en el borde o fuera de un polígono mediante Ray Casting (Jordan Curve Theorem).
     * Complejidad: O(V) donde V es el número de vértices del polígono.
     */
    PointLocation pointInPolygon(const Point& pt, const Polygon& poly);

    /**
     * @brief Helper que retorna true si el punto está estrictamente en el interior del polígono.
     */
    bool isPointStrictlyInside(const Point& pt, const Polygon& poly);

    /**
     * @brief Helper que retorna true si el punto está en el interior o en el borde del polígono.
     */
    bool isPointInsideOrBoundary(const Point& pt, const Polygon& poly);

    /**
     * @brief Valida si una trayectoria entre u y v interseca o penetra el interior de un polígono de exclusión.
     * Requisito operativo RF04 / Sec 1.1: segmento(u, v) ∩ interior(poly) = ∅.
     */
    bool segmentIntersectsPolygonInterior(const Segment& seg, const Polygon& poly);

    /**
     * @brief Valida si una trayectoria entre u y v es factible frente a una colección de obstáculos poligonales.
     */
    bool isFlightPathClear(const Segment& seg, const std::vector<Polygon>& obstacles);

} // namespace GeometryUtils

} // namespace geodrone

#endif // GEODRONE_GEOMETRYUTILS_HPP
