#include "../src/geometry/Point.hpp"
#include "../src/geometry/Segment.hpp"
#include "../src/geometry/Polygon.hpp"
#include "../src/geometry/GeometryUtils.hpp"

#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

using namespace geodrone;

void testCCWAndOrientation() {
    std::cout << "[TEST] 1. Giro horario, antihorario y colinealidad (CCW)..." << std::endl;

    Point a(0.0, 0.0);
    Point b(4.0, 0.0);

    // Punto hacia arriba -> Giro antihorario (izquierda)
    Point c_left(2.0, 3.0);
    assert(GeometryUtils::ccw(a, b, c_left) == 1);
    assert(GeometryUtils::getOrientation(a, b, c_left) == Orientation::COUNTER_CLOCKWISE);

    // Punto hacia abajo -> Giro horario (derecha)
    Point c_right(2.0, -3.0);
    assert(GeometryUtils::ccw(a, b, c_right) == -1);
    assert(GeometryUtils::getOrientation(a, b, c_right) == Orientation::CLOCKWISE);

    // Puntos estrictamente colineales
    Point c_collinear(8.0, 0.0);
    Point c_between(2.0, 0.0);
    Point c_behind(-2.0, 0.0);
    assert(GeometryUtils::ccw(a, b, c_collinear) == 0);
    assert(GeometryUtils::ccw(a, b, c_between) == 0);
    assert(GeometryUtils::ccw(a, b, c_behind) == 0);
    assert(GeometryUtils::getOrientation(a, b, c_between) == Orientation::COLLINEAR);

    // Casos con precisión épsilon (1e-10 de perturbación debe considerarse colineal con EPS = 1e-9)
    Point c_near_collinear(2.0, 1e-10);
    assert(GeometryUtils::ccw(a, b, c_near_collinear) == 0);

    std::cout << "  -> PASSED: Orientacion CCW verificada exitosamente.\n" << std::endl;
}

void testSegmentIntersections() {
    std::cout << "[TEST] 2. Interseccion propia e impropia de segmentos..." << std::endl;

    // 2.1 Intersección propia en cruz perfecta
    Segment s1(Point(0.0, 2.0), Point(4.0, 2.0));
    Segment s2(Point(2.0, 0.0), Point(2.0, 4.0));
    assert(GeometryUtils::segmentsIntersect(s1, s2, false) == true);
    assert(GeometryUtils::properIntersection(s1, s2) == true);

    // 2.2 Segmentos en forma de 'T' (intersección impropia en un extremo)
    Segment s_horiz(Point(0.0, 0.0), Point(4.0, 0.0));
    Segment s_touch(Point(2.0, 0.0), Point(2.0, 3.0));
    assert(GeometryUtils::segmentsIntersect(s_horiz, s_touch, false) == true);
    // En intersección propia no debe contar porque toca en el borde
    assert(GeometryUtils::properIntersection(s_horiz, s_touch) == false);

    // 2.3 Segmentos colineales solapados
    Segment s_col1(Point(0.0, 0.0), Point(4.0, 0.0));
    Segment s_col2(Point(2.0, 0.0), Point(6.0, 0.0));
    assert(GeometryUtils::segmentsIntersect(s_col1, s_col2, false) == true);
    assert(GeometryUtils::properIntersection(s_col1, s_col2) == false);

    // 2.4 Segmentos colineales disjuntos
    Segment s_col3(Point(5.0, 0.0), Point(8.0, 0.0));
    assert(GeometryUtils::segmentsIntersect(s_col1, s_col3, false) == false);
    assert(GeometryUtils::properIntersection(s_col1, s_col3) == false);

    // 2.5 Segmentos paralelos no colineales
    Segment s_par1(Point(0.0, 0.0), Point(4.0, 0.0));
    Segment s_par2(Point(0.0, 1.0), Point(4.0, 1.0));
    assert(GeometryUtils::segmentsIntersect(s_par1, s_par2, false) == false);
    assert(GeometryUtils::properIntersection(s_par1, s_par2) == false);

    std::cout << "  -> PASSED: Interseccion de segmentos verificada exitosamente.\n" << std::endl;
}

void testPointInPolygon() {
    std::cout << "[TEST] 3. Prueba de pertenencia Punto en Poligono (Ray Casting)..." << std::endl;

    // Cuadrado unitario de 10x10: [0, 10] x [0, 10]
    Polygon square(1, "Obstaculo_Cuadrado");
    square.addVertex(Point(0.0, 0.0));
    square.addVertex(Point(10.0, 0.0));
    square.addVertex(Point(10.0, 10.0));
    square.addVertex(Point(0.0, 10.0));

    // Puntos estrictamente interiores
    assert(GeometryUtils::pointInPolygon(Point(5.0, 5.0), square) == PointLocation::INSIDE);
    assert(GeometryUtils::isPointStrictlyInside(Point(5.0, 5.0), square) == true);
    assert(GeometryUtils::pointInPolygon(Point(1.0, 1.0), square) == PointLocation::INSIDE);
    assert(GeometryUtils::pointInPolygon(Point(9.9, 9.9), square) == PointLocation::INSIDE);

    // Puntos estrictamente exteriores
    assert(GeometryUtils::pointInPolygon(Point(-1.0, 5.0), square) == PointLocation::OUTSIDE);
    assert(GeometryUtils::isPointStrictlyInside(Point(-1.0, 5.0), square) == false);
    assert(GeometryUtils::pointInPolygon(Point(15.0, 5.0), square) == PointLocation::OUTSIDE);
    assert(GeometryUtils::pointInPolygon(Point(5.0, -0.5), square) == PointLocation::OUTSIDE);
    assert(GeometryUtils::pointInPolygon(Point(5.0, 10.5), square) == PointLocation::OUTSIDE);

    // Puntos sobre el borde (Boundary)
    assert(GeometryUtils::pointInPolygon(Point(0.0, 5.0), square) == PointLocation::BOUNDARY);
    assert(GeometryUtils::pointInPolygon(Point(10.0, 5.0), square) == PointLocation::BOUNDARY);
    assert(GeometryUtils::pointInPolygon(Point(5.0, 0.0), square) == PointLocation::BOUNDARY);
    assert(GeometryUtils::pointInPolygon(Point(5.0, 10.0), square) == PointLocation::BOUNDARY);

    // Vértices del polígono
    assert(GeometryUtils::pointInPolygon(Point(0.0, 0.0), square) == PointLocation::BOUNDARY);
    assert(GeometryUtils::pointInPolygon(Point(10.0, 10.0), square) == PointLocation::BOUNDARY);

    // Polígono no convexo (Forma de 'L')
    Polygon polyL(2, "Edificio_L");
    polyL.addVertex(Point(0.0, 0.0));
    polyL.addVertex(Point(6.0, 0.0));
    polyL.addVertex(Point(6.0, 2.0));
    polyL.addVertex(Point(2.0, 2.0));
    polyL.addVertex(Point(2.0, 6.0));
    polyL.addVertex(Point(0.0, 6.0));

    // Punto en la hendidura exterior de la 'L' (debe ser OUTSIDE)
    assert(GeometryUtils::pointInPolygon(Point(4.0, 4.0), polyL) == PointLocation::OUTSIDE);
    // Punto en el interior de la 'L'
    assert(GeometryUtils::pointInPolygon(Point(1.0, 1.0), polyL) == PointLocation::INSIDE);
    assert(GeometryUtils::pointInPolygon(Point(1.0, 4.0), polyL) == PointLocation::INSIDE);
    assert(GeometryUtils::pointInPolygon(Point(4.0, 1.0), polyL) == PointLocation::INSIDE);

    std::cout << "  -> PASSED: Point in Polygon (Ray Casting) verificado exitosamente.\n" << std::endl;
}

void testPolygonMetrics() {
    std::cout << "[TEST] 4. Metricas de Poligonos (Area Shoelace y Perimetro)..." << std::endl;

    Polygon rect(1, "Rectangulo");
    rect.addVertex(Point(0.0, 0.0));
    rect.addVertex(Point(8.0, 0.0));
    rect.addVertex(Point(8.0, 5.0));
    rect.addVertex(Point(0.0, 5.0));

    assert(std::abs(rect.area() - 40.0) < EPS);
    assert(std::abs(rect.perimeter() - 26.0) < EPS);

    std::cout << "  -> PASSED: Area y perimetro calculados con precision.\n" << std::endl;
}

void testTrajectoryValidation() {
    std::cout << "[TEST] 5. Validacion de Trayectorias de Vuelo vs Obstaculos..." << std::endl;

    Polygon obstacle(1, "Edificio");
    obstacle.addVertex(Point(2.0, 2.0));
    obstacle.addVertex(Point(6.0, 2.0));
    obstacle.addVertex(Point(6.0, 6.0));
    obstacle.addVertex(Point(2.0, 6.0));

    // 5.1 Trayectoria que atraviesa directamente el obstáculo (Infactible)
    Segment invalidPath(Point(0.0, 4.0), Point(8.0, 4.0));
    assert(GeometryUtils::segmentIntersectsPolygonInterior(invalidPath, obstacle) == true);

    // 5.2 Trayectoria totalmente libre y separada (Factible)
    Segment validPath(Point(0.0, 0.0), Point(8.0, 0.0));
    assert(GeometryUtils::segmentIntersectsPolygonInterior(validPath, obstacle) == false);

    // 5.3 Trayectoria tangencial que roza exactamente un vértice por el exterior
    Segment tangentPath(Point(0.0, 2.0), Point(2.0, 0.0));
    assert(GeometryUtils::segmentIntersectsPolygonInterior(tangentPath, obstacle) == false);

    std::cout << "  -> PASSED: Validacion de trayectorias verificada exitosamente.\n" << std::endl;
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - SUITE DE PRUEBAS GEOMETRICAS    " << std::endl;
    std::cout << "        USIL - Programacion Competitiva (2026)          " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    testCCWAndOrientation();
    testSegmentIntersections();
    testPointInPolygon();
    testPolygonMetrics();
    testTrajectoryValidation();

    std::cout << "=========================================================" << std::endl;
    std::cout << "   ¡TODAS LAS PRUEBAS GEOMETRICAS PASARON CON EXITO!    " << std::endl;
    std::cout << "=========================================================" << std::endl;

    return 0;
}
