#include "GeometryUtils.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace geodrone {
namespace GeometryUtils {

int ccw(const Point& a, const Point& b, const Point& c) {
    double cp = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (cp > EPS) return 1;    // Antihorario / izquierda
    if (cp < -EPS) return -1;  // Horario / derecha
    return 0;                  // Colineal
}

Orientation getOrientation(const Point& a, const Point& b, const Point& c) {
    int res = ccw(a, b, c);
    if (res > 0) return Orientation::COUNTER_CLOCKWISE;
    if (res < 0) return Orientation::CLOCKWISE;
    return Orientation::COLLINEAR;
}

bool onSegment(const Point& p, const Point& a, const Point& b) {
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

bool onSegment(const Point& p, const Segment& s) {
    return onSegment(p, s.a, s.b);
}

bool segmentsIntersect(const Segment& s1, const Segment& s2, bool properOnly) {
    const Point& p1 = s1.a;
    const Point& q1 = s1.b;
    const Point& p2 = s2.a;
    const Point& q2 = s2.b;

    int o1 = ccw(p1, q1, p2);
    int o2 = ccw(p1, q1, q2);
    int o3 = ccw(p2, q2, p1);
    int o4 = ccw(p2, q2, q1);

    // Intersección propia estricta (en cruz, sin que ningún extremo toque el otro segmento)
    bool proper = (o1 * o2 < 0) && (o3 * o4 < 0);
    if (properOnly) {
        return proper;
    }
    if (proper) {
        return true;
    }

    // Casos especiales: puntos colineales donde un extremo reside dentro del otro segmento
    if (o1 == 0 && onSegment(p2, p1, q1)) return true;
    if (o2 == 0 && onSegment(q2, p1, q1)) return true;
    if (o3 == 0 && onSegment(p1, p2, q2)) return true;
    if (o4 == 0 && onSegment(q1, p2, q2)) return true;

    return false;
}

bool properIntersection(const Segment& s1, const Segment& s2) {
    return segmentsIntersect(s1, s2, true);
}

PointLocation pointInPolygon(const Point& pt, const Polygon& poly) {
    int n = poly.size();
    if (n < 3) return PointLocation::OUTSIDE;

    // 1. Validar primero si el punto reside sobre alguna arista perimetral o vértice
    for (int i = 0; i < n; ++i) {
        int next = (i + 1) % n;
        if (onSegment(pt, poly[i], poly[next])) {
            return PointLocation::BOUNDARY;
        }
    }

    // 2. Algoritmo de Ray Casting (Teorema de la curva de Jordan)
    // Se proyecta un rayo horizontal hacia x -> +inf
    int crossings = 0;
    for (int i = 0; i < n; ++i) {
        Point v1 = poly[i];
        Point v2 = poly[(i + 1) % n];

        // Normalizar para que v1 tenga menor o igual coordenada Y
        if (v1.y > v2.y) {
            std::swap(v1, v2);
        }

        // Evaluar si el rayo horizontal en pt.y cruza el semi-intervalo [v1.y, v2.y)
        // La condición semi-abierta previene contar dos veces cuando el rayo cruza un vértice común.
        if (pt.y >= v1.y - EPS && pt.y < v2.y - EPS) {
            double dy = v2.y - v1.y;
            if (std::abs(dy) > EPS) {
                double xIntersection = v1.x + (pt.y - v1.y) * (v2.x - v1.x) / dy;
                if (xIntersection > pt.x + EPS) {
                    crossings++;
                }
            }
        }
    }

    return (crossings % 2 == 1) ? PointLocation::INSIDE : PointLocation::OUTSIDE;
}

bool isPointStrictlyInside(const Point& pt, const Polygon& poly) {
    return pointInPolygon(pt, poly) == PointLocation::INSIDE;
}

bool isPointInsideOrBoundary(const Point& pt, const Polygon& poly) {
    PointLocation loc = pointInPolygon(pt, poly);
    return loc == PointLocation::INSIDE || loc == PointLocation::BOUNDARY;
}

bool segmentIntersectsPolygonInterior(const Segment& seg, const Polygon& poly) {
    int n = poly.size();
    if (n < 3) return false;

    // Caso de punto degenerado
    if (seg.a == seg.b) {
        return isPointStrictlyInside(seg.a, poly);
    }

    // 1. Si el segmento interseca propiamente (en cruz) cualquier arista del polígono,
    // forzosamente penetra o atraviesa el interior.
    auto edges = poly.getEdges();
    for (const auto& edge : edges) {
        if (properIntersection(seg, edge)) {
            return true;
        }
    }

    // 2. Si no hay intersección propia en cruz, podría ser una cuerda interna
    // (ej. conectando dos vértices del polígono a través del interior) o un segmento
    // que toca un vértice tangencialmente.
    // Analizamos los puntos medios muestreados a lo largo del segmento.
    // Recolectamos todas las proyecciones t en [0, 1] donde el segmento toca vértices o aristas.
    std::vector<double> tSamples = {0.0, 1.0};

    Point dir = seg.b - seg.a;
    double segLenSq = dir.x * dir.x + dir.y * dir.y;

    if (segLenSq > EPS) {
        for (int i = 0; i < n; ++i) {
            Point p = poly[i];
            if (onSegment(p, seg)) {
                double t = ((p.x - seg.a.x) * dir.x + (p.y - seg.a.y) * dir.y) / segLenSq;
                if (t > EPS && t < 1.0 - EPS) {
                    tSamples.push_back(t);
                }
            }
        }
    }

    std::sort(tSamples.begin(), tSamples.end());
    tSamples.erase(std::unique(tSamples.begin(), tSamples.end(), [](double a, double b) {
        return std::abs(a - b) < EPS;
    }), tSamples.end());

    // Evaluamos el punto medio de cada sub-intervalo libre
    for (size_t i = 0; i + 1 < tSamples.size(); ++i) {
        double midT = (tSamples[i] + tSamples[i + 1]) * 0.5;
        Point testPoint = seg.a + dir * midT;
        if (isPointStrictlyInside(testPoint, poly)) {
            return true;
        }
    }

    return false;
}

bool isFlightPathClear(const Segment& seg, const std::vector<Polygon>& obstacles) {
    for (const auto& poly : obstacles) {
        if (segmentIntersectsPolygonInterior(seg, poly)) {
            return false;
        }
    }
    return true;
}

} // namespace GeometryUtils
} // namespace geodrone
