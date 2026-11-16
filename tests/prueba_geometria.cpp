// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/geometria/Punto.hpp"
#include "../src/geometria/Segmento.hpp"
#include "../src/geometria/Poligono.hpp"
#include "../src/geometria/Geometria.hpp"
#include "../src/geometria/EnvolventeConvexa.hpp"

#include <algorithm>
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

using namespace geodrone;

void probarOrientacion() {
    std::cout << "[PRUEBA] 1. Giro horario, antihorario y colinealidad (CCW)..." << std::endl;

    Punto a(0.0, 0.0);
    Punto b(4.0, 0.0);

    // Punto hacia arriba -> Giro antihorario (izquierda)
    Punto cIzquierda(2.0, 3.0);
    assert(Geometria::orientacion(a, b, cIzquierda) == 1);

    // Punto hacia abajo -> Giro horario (derecha)
    Punto cDerecha(2.0, -3.0);
    assert(Geometria::orientacion(a, b, cDerecha) == -1);

    // Puntos estrictamente colineales
    Punto cColineal(8.0, 0.0);
    Punto cEntre(2.0, 0.0);
    Punto cDetras(-2.0, 0.0);
    assert(Geometria::orientacion(a, b, cColineal) == 0);
    assert(Geometria::orientacion(a, b, cEntre) == 0);
    assert(Geometria::orientacion(a, b, cDetras) == 0);

    // Casos con precisión épsilon (1e-10 de perturbación debe considerarse colineal con EPS = 1e-9)
    Punto cCasiColineal(2.0, 1e-10);
    assert(Geometria::orientacion(a, b, cCasiColineal) == 0);

    std::cout << "  -> OK\n" << std::endl;
}

void probarInterseccionSegmentos() {
    std::cout << "[PRUEBA] 2. Interseccion propia e impropia de segmentos..." << std::endl;

    // 2.1 Intersección propia en cruz perfecta
    Segmento s1(Punto(0.0, 2.0), Punto(4.0, 2.0));
    Segmento s2(Punto(2.0, 0.0), Punto(2.0, 4.0));
    assert(Geometria::segmentosSeIntersecan(s1, s2, false) == true);
    assert(Geometria::interseccionPropia(s1, s2) == true);

    // 2.2 Segmentos en forma de 'T' (intersección impropia en un extremo)
    Segmento sHorizontal(Punto(0.0, 0.0), Punto(4.0, 0.0));
    Segmento sToca(Punto(2.0, 0.0), Punto(2.0, 3.0));
    assert(Geometria::segmentosSeIntersecan(sHorizontal, sToca, false) == true);
    // En intersección propia no debe contar porque toca en el borde
    assert(Geometria::interseccionPropia(sHorizontal, sToca) == false);

    // 2.3 Segmentos colineales solapados
    Segmento sColineal1(Punto(0.0, 0.0), Punto(4.0, 0.0));
    Segmento sColineal2(Punto(2.0, 0.0), Punto(6.0, 0.0));
    assert(Geometria::segmentosSeIntersecan(sColineal1, sColineal2, false) == true);
    assert(Geometria::interseccionPropia(sColineal1, sColineal2) == false);

    // 2.4 Segmentos colineales disjuntos
    Segmento sColineal3(Punto(5.0, 0.0), Punto(8.0, 0.0));
    assert(Geometria::segmentosSeIntersecan(sColineal1, sColineal3, false) == false);
    assert(Geometria::interseccionPropia(sColineal1, sColineal3) == false);

    // 2.5 Segmentos paralelos no colineales
    Segmento sParalelo1(Punto(0.0, 0.0), Punto(4.0, 0.0));
    Segmento sParalelo2(Punto(0.0, 1.0), Punto(4.0, 1.0));
    assert(Geometria::segmentosSeIntersecan(sParalelo1, sParalelo2, false) == false);
    assert(Geometria::interseccionPropia(sParalelo1, sParalelo2) == false);

    std::cout << "  -> OK\n" << std::endl;
}

void probarPuntoEnPoligono() {
    std::cout << "[PRUEBA] 3. Prueba de pertenencia Punto en Poligono (Ray Casting)..." << std::endl;

    // Cuadrado unitario de 10x10: [0, 10] x [0, 10]
    Poligono cuadrado(1, "Obstaculo_Cuadrado");
    cuadrado.agregarVertice(Punto(0.0, 0.0));
    cuadrado.agregarVertice(Punto(10.0, 0.0));
    cuadrado.agregarVertice(Punto(10.0, 10.0));
    cuadrado.agregarVertice(Punto(0.0, 10.0));

    // Puntos estrictamente interiores
    assert(Geometria::ubicacionEnPoligono(Punto(5.0, 5.0), cuadrado) == UbicacionPunto::DENTRO);
    assert(Geometria::estaEstrictamenteDentro(Punto(5.0, 5.0), cuadrado) == true);
    assert(Geometria::ubicacionEnPoligono(Punto(1.0, 1.0), cuadrado) == UbicacionPunto::DENTRO);
    assert(Geometria::ubicacionEnPoligono(Punto(9.9, 9.9), cuadrado) == UbicacionPunto::DENTRO);

    // Puntos estrictamente exteriores
    assert(Geometria::ubicacionEnPoligono(Punto(-1.0, 5.0), cuadrado) == UbicacionPunto::FUERA);
    assert(Geometria::estaEstrictamenteDentro(Punto(-1.0, 5.0), cuadrado) == false);
    assert(Geometria::ubicacionEnPoligono(Punto(15.0, 5.0), cuadrado) == UbicacionPunto::FUERA);
    assert(Geometria::ubicacionEnPoligono(Punto(5.0, -0.5), cuadrado) == UbicacionPunto::FUERA);
    assert(Geometria::ubicacionEnPoligono(Punto(5.0, 10.5), cuadrado) == UbicacionPunto::FUERA);

    // Puntos sobre el borde (Boundary)
    assert(Geometria::ubicacionEnPoligono(Punto(0.0, 5.0), cuadrado) == UbicacionPunto::BORDE);
    assert(Geometria::ubicacionEnPoligono(Punto(10.0, 5.0), cuadrado) == UbicacionPunto::BORDE);
    assert(Geometria::ubicacionEnPoligono(Punto(5.0, 0.0), cuadrado) == UbicacionPunto::BORDE);
    assert(Geometria::ubicacionEnPoligono(Punto(5.0, 10.0), cuadrado) == UbicacionPunto::BORDE);

    // Vértices del polígono
    assert(Geometria::ubicacionEnPoligono(Punto(0.0, 0.0), cuadrado) == UbicacionPunto::BORDE);
    assert(Geometria::ubicacionEnPoligono(Punto(10.0, 10.0), cuadrado) == UbicacionPunto::BORDE);

    // Polígono no convexo (Forma de 'L')
    Poligono poligonoL(2, "Edificio_L");
    poligonoL.agregarVertice(Punto(0.0, 0.0));
    poligonoL.agregarVertice(Punto(6.0, 0.0));
    poligonoL.agregarVertice(Punto(6.0, 2.0));
    poligonoL.agregarVertice(Punto(2.0, 2.0));
    poligonoL.agregarVertice(Punto(2.0, 6.0));
    poligonoL.agregarVertice(Punto(0.0, 6.0));

    // Punto en la hendidura exterior de la 'L' (debe ser FUERA)
    assert(Geometria::ubicacionEnPoligono(Punto(4.0, 4.0), poligonoL) == UbicacionPunto::FUERA);
    // Punto en el interior de la 'L'
    assert(Geometria::ubicacionEnPoligono(Punto(1.0, 1.0), poligonoL) == UbicacionPunto::DENTRO);
    assert(Geometria::ubicacionEnPoligono(Punto(1.0, 4.0), poligonoL) == UbicacionPunto::DENTRO);
    assert(Geometria::ubicacionEnPoligono(Punto(4.0, 1.0), poligonoL) == UbicacionPunto::DENTRO);

    std::cout << "  -> OK\n" << std::endl;
}

void probarMetricasPoligono() {
    std::cout << "[PRUEBA] 4. Metricas de Poligonos (Area Shoelace y Perimetro)..." << std::endl;

    Poligono rectangulo(1, "Rectangulo");
    rectangulo.agregarVertice(Punto(0.0, 0.0));
    rectangulo.agregarVertice(Punto(8.0, 0.0));
    rectangulo.agregarVertice(Punto(8.0, 5.0));
    rectangulo.agregarVertice(Punto(0.0, 5.0));

    assert(std::abs(rectangulo.area() - 40.0) < EPS);
    assert(std::abs(rectangulo.perimetro() - 26.0) < EPS);

    std::cout << "  -> OK\n" << std::endl;
}

void probarValidacionTramos() {
    std::cout << "[PRUEBA] 5. Validacion de Trayectorias de Vuelo vs Obstaculos..." << std::endl;

    Poligono obstaculo(1, "Edificio");
    obstaculo.agregarVertice(Punto(2.0, 2.0));
    obstaculo.agregarVertice(Punto(6.0, 2.0));
    obstaculo.agregarVertice(Punto(6.0, 6.0));
    obstaculo.agregarVertice(Punto(2.0, 6.0));

    // 5.1 Trayectoria que atraviesa directamente el obstáculo (Infactible)
    Segmento tramoInvalido(Punto(0.0, 4.0), Punto(8.0, 4.0));
    assert(Geometria::segmentoCruzaInterior(tramoInvalido, obstaculo) == true);

    // 5.2 Trayectoria totalmente libre y separada (Factible)
    Segmento tramoValido(Punto(0.0, 0.0), Punto(8.0, 0.0));
    assert(Geometria::segmentoCruzaInterior(tramoValido, obstaculo) == false);

    // 5.3 Trayectoria tangencial que roza exactamente el vértice (2,2) por el exterior (Factible)
    Segmento tramoTangente(Punto(0.0, 4.0), Punto(4.0, 0.0));
    assert(Geometria::segmentoCruzaInterior(tramoTangente, obstaculo) == false);

    // 5.4 Diagonal que entra y sale por los vértices (2,2) y (6,6): no cruza ninguna arista
    // en forma propia, pero recorre el interior (Infactible)
    Segmento tramoDiagonal(Punto(0.0, 0.0), Punto(8.0, 8.0));
    assert(Geometria::segmentoCruzaInterior(tramoDiagonal, obstaculo) == true);

    // 5.5 Trayectoria que vuela sobre el borde izquierdo x = 2 (Factible: tocar el borde está permitido)
    Segmento tramoBorde(Punto(2.0, 0.0), Punto(2.0, 8.0));
    assert(Geometria::segmentoCruzaInterior(tramoBorde, obstaculo) == false);

    std::cout << "  -> OK\n" << std::endl;
}

void probarEnvolventeConvexa() {
    std::cout << "[PRUEBA] 6. Envolvente convexa (monotone chain)..." << std::endl;

    // Cuadrado con puntos interiores, puntos colineales en los bordes y un duplicado.
    std::vector<Punto> puntos = {
        Punto(0, 0), Punto(4, 0), Punto(4, 4), Punto(0, 4),   // Esquinas
        Punto(2, 2), Punto(1, 3), Punto(3, 1),                // Interiores
        Punto(2, 0), Punto(4, 2), Punto(0, 2),                // Colineales sobre bordes
        Punto(4, 4)                                           // Duplicado
    };
    std::vector<Punto> envolvente = envolventeConvexa(puntos);
    assert(envolvente.size() == 4);

    // Las cuatro esquinas, en sentido antihorario: el área con signo es positiva.
    Poligono poligonoEnvolvente(1, envolvente);
    assert(std::abs(poligonoEnvolvente.areaConSigno() - 16.0) < EPS);
    for (const Punto& esquina : {Punto(0, 0), Punto(4, 0), Punto(4, 4), Punto(0, 4)}) {
        assert(std::find(envolvente.begin(), envolvente.end(), esquina) != envolvente.end());
    }

    // Todos los puntos de entrada quedan dentro o sobre el borde de la envolvente.
    for (const Punto& p : puntos) {
        assert(Geometria::ubicacionEnPoligono(p, poligonoEnvolvente) != UbicacionPunto::FUERA);
    }

    // Casos degenerados: puntos colineales o menos de 3 puntos no forman polígono.
    assert(envolventeConvexa({Punto(0, 0), Punto(1, 1), Punto(2, 2), Punto(3, 3)}).size() < 3);
    assert(envolventeConvexa({Punto(5, 5), Punto(5, 5), Punto(6, 7)}).size() < 3);

    std::cout << "  -> OK\n" << std::endl;
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - SUITE DE PRUEBAS GEOMETRICAS    " << std::endl;
    std::cout << "        USIL - Programacion Competitiva (2026)          " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    probarOrientacion();
    probarInterseccionSegmentos();
    probarPuntoEnPoligono();
    probarMetricasPoligono();
    probarValidacionTramos();
    probarEnvolventeConvexa();

    std::cout << "=========================================================" << std::endl;
    std::cout << "   ¡TODAS LAS PRUEBAS GEOMETRICAS PASARON CON EXITO!    " << std::endl;
    std::cout << "=========================================================" << std::endl;

    return 0;
}
