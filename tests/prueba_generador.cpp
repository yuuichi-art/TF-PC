// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/escenario/GeneradorDatasets.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"
#include "../src/geometria/Geometria.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;

const Categoria CATEGORIAS_GENERADAS[] = {
    Categoria::SMALL, Categoria::MEDIUM, Categoria::LARGE, Categoria::EXTREME
};

std::string aTexto(const Escenario& escenario) {
    std::ostringstream salida;
    ArchivoEscenario::escribir(escenario, salida);
    return salida.str();
}

// Solo la geometría: las pruebas de este archivo no necesitan la batería, que construye el
// grafo de misión completo y haría lentas las pruebas con muchos escenarios Extreme.
Escenario soloGeometria(Categoria categoria, uint64_t semilla) {
    return GeneradorDatasets::generarGeometria(GeneradorDatasets::rangosDe(categoria), categoria, semilla);
}

bool esEntero(double valor) {
    return valor == std::floor(valor);
}

bool dentroDelArea(const Escenario& s, const Punto& p) {
    return p.x >= 0 && p.x <= s.anchoArea && p.y >= 0 && p.y <= s.altoArea;
}

// Dos polígonos convexos se tocan o se solapan si alguna arista de uno corta (o toca) una
// arista del otro, o si un vértice de uno está dentro o sobre el borde del otro.
bool poligonosSeTocan(const Poligono& a, const Poligono& b) {
    for (const Segmento& ea : a.aristas()) {
        for (const Segmento& eb : b.aristas()) {
            if (Geometria::segmentosSeIntersecan(ea, eb)) return true;
        }
    }
    return Geometria::ubicacionEnPoligono(a[0], b) != UbicacionPunto::FUERA ||
           Geometria::ubicacionEnPoligono(b[0], a) != UbicacionPunto::FUERA;
}

void probarMismaSemillaMismoEscenario() {
    std::cout << "[TEST] 1. La misma semilla produce el mismo escenario..." << std::endl;

    for (Categoria categoria : CATEGORIAS_GENERADAS) {
        for (uint64_t semilla : {1ULL, 7ULL, 123456789ULL}) {
            std::string primero = aTexto(soloGeometria(categoria, semilla));
            std::string segundo = aTexto(soloGeometria(categoria, semilla));
            assert(primero == segundo);
        }
        // Semillas distintas dan escenarios distintos.
        assert(aTexto(soloGeometria(categoria, 1)) != aTexto(soloGeometria(categoria, 2)));
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarTamanosDentroDeCategoria() {
    std::cout << "[TEST] 2. Cantidades dentro de los rangos de cada categoria..." << std::endl;

    const int maxVerticesObstaculos[] = {30, 100, 400, 1500};
    int indice = 0;
    for (Categoria categoria : CATEGORIAS_GENERADAS) {
        RangosCategoria rangos = GeneradorDatasets::rangosDe(categoria);
        for (uint64_t semilla = 1; semilla <= 10; ++semilla) {
            Escenario s = soloGeometria(categoria, semilla);
            assert(s.categoria == categoria && s.semilla == semilla);
            assert(s.numClientes() >= rangos.minClientes && s.numClientes() <= rangos.maxClientes);
            assert(s.numEstaciones() >= rangos.minEstaciones && s.numEstaciones() <= rangos.maxEstaciones);
            assert(s.numObstaculos() >= rangos.minObstaculos && s.numObstaculos() <= rangos.maxObstaculos);
            assert(s.totalVerticesObstaculos() <= maxVerticesObstaculos[indice]);
            for (const Poligono& poligono : s.obstaculos) {
                assert(poligono.numVertices() >= 3 && poligono.numVertices() <= rangos.maxPuntosPorObstaculo);
            }
        }
        ++indice;
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarGeometriaValida() {
    std::cout << "[TEST] 3. Obstaculos convexos sin solape y puntos libres y enteros..." << std::endl;

    for (Categoria categoria : CATEGORIAS_GENERADAS) {
        for (uint64_t semilla = 1; semilla <= 5; ++semilla) {
            Escenario s = soloGeometria(categoria, semilla);

            // Obstáculos: convexos (antihorarios, sin giros a la derecha), dentro del área,
            // con coordenadas enteras y sin tocarse entre sí.
            for (const Poligono& poligono : s.obstaculos) {
                int n = poligono.numVertices();
                for (int i = 0; i < n; ++i) {
                    const Punto& v = poligono[i];
                    assert(esEntero(v.x) && esEntero(v.y) && dentroDelArea(s, v));
                    assert(Geometria::orientacion(poligono[i], poligono[(i + 1) % n], poligono[(i + 2) % n]) > 0);
                }
            }
            for (int i = 0; i < s.numObstaculos(); ++i) {
                for (int j = i + 1; j < s.numObstaculos(); ++j) {
                    assert(!poligonosSeTocan(s.obstaculos[i], s.obstaculos[j]));
                }
            }

            // Base, estaciones y clientes: enteros, dentro del área, fuera de los obstáculos
            // y sin repetir ningún punto.
            std::vector<Punto> puntos = s.estaciones;
            puntos.insert(puntos.end(), s.clientes.begin(), s.clientes.end());
            puntos.push_back(s.base);
            for (size_t i = 0; i < puntos.size(); ++i) {
                assert(esEntero(puntos[i].x) && esEntero(puntos[i].y) && dentroDelArea(s, puntos[i]));
                for (const Poligono& poligono : s.obstaculos) {
                    assert(Geometria::ubicacionEnPoligono(puntos[i], poligono) == UbicacionPunto::FUERA);
                }
                for (size_t j = i + 1; j < puntos.size(); ++j) {
                    assert(puntos[i] != puntos[j]);
                }
            }
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarAdversarialRechazado() {
    std::cout << "[TEST] 4. La categoria ADVERSARIAL no se genera..." << std::endl;

    bool rechazado = false;
    try {
        GeneradorDatasets::generar(Categoria::ADVERSARIAL, 1);
    } catch (const std::invalid_argument&) {
        rechazado = true;
    }
    assert(rechazado);

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarConjuntoBenchmark() {
    std::cout << "[TEST] 5. Set de benchmark guardado y releido desde archivo..." << std::endl;

    const std::string carpeta = "tmp_prueba_generator";
    std::filesystem::remove_all(carpeta);

    // Una semilla por categoría y sin el peor caso: cada escenario completo construye su grafo
    // de misión, y el Extreme al máximo tarda varios segundos (más aún en Debug).
    std::vector<std::string> rutas = GeneradorDatasets::generarConjuntoBenchmark(carpeta, 1, false);
    assert(rutas.size() == 4);
    assert(rutas.front() == carpeta + "/small_1.txt");
    assert(rutas.back() == carpeta + "/extreme_1.txt");

    // Lo guardado es exactamente lo que genera la semilla, y tiene batería calculada.
    Escenario desdeArchivo = ArchivoEscenario::cargar(carpeta + "/medium_1.txt");
    assert(aTexto(desdeArchivo) == aTexto(GeneradorDatasets::generar(Categoria::MEDIUM, 1)));
    assert(desdeArchivo.bateria >= 1.0 && esEntero(desdeArchivo.bateria));

    // La geometría del escenario completo es la misma que la de generarGeometria.
    Escenario completo = GeneradorDatasets::generar(Categoria::SMALL, 3);
    Escenario sinBateria = soloGeometria(Categoria::SMALL, 3);
    sinBateria.bateria = completo.bateria;
    assert(aTexto(completo) == aTexto(sinBateria));

    // El peor caso fija cada cantidad en el máximo del rango.
    for (Categoria categoria : CATEGORIAS_GENERADAS) {
        RangosCategoria rango = GeneradorDatasets::rangosDe(categoria);
        Escenario peor = GeneradorDatasets::generarGeometria(GeneradorDatasets::rangosPeorCaso(categoria), categoria, 1);
        assert(peor.numClientes() == rango.maxClientes);
        assert(peor.numEstaciones() == rango.maxEstaciones);
        assert(peor.numObstaculos() == rango.maxObstaculos);
    }

    std::filesystem::remove_all(carpeta);

    std::cout << "  -> PASSED\n" << std::endl;
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DEL GENERADOR            " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    probarMismaSemillaMismoEscenario();
    probarTamanosDentroDeCategoria();
    probarGeometriaValida();
    probarAdversarialRechazado();
    probarConjuntoBenchmark();

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DEL GENERADOR PASARON CON EXITO     " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
