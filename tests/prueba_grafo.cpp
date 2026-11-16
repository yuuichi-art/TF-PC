// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/grafo/GrafoMision.hpp"
#include "../src/grafo/Dijkstra.hpp"
#include "../src/escenario/GeneradorDatasets.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"
#include "../src/geometria/Geometria.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace geodrone;

const double TOLERANCIA = 1e-6;

// Escenarios de prueba: generados (Small y Medium) y los adversariales hechos a mano.
std::vector<Escenario> escenariosDeMuestra(const std::string& carpetaAdversarial) {
    std::vector<Escenario> escenarios;
    for (uint64_t semilla = 1; semilla <= 3; ++semilla) {
        escenarios.push_back(GeneradorDatasets::generar(Categoria::SMALL, semilla));
        escenarios.push_back(GeneradorDatasets::generar(Categoria::MEDIUM, semilla));
    }
    for (const char* nombre : {"adv_collinear", "adv_enclosed", "adv_corridor", "adv_concave_trap",
                               "adv_unreachable_battery"}) {
        escenarios.push_back(ArchivoEscenario::cargar(carpetaAdversarial + "/" + nombre + ".txt"));
    }
    return escenarios;
}

GrafoVisibilidad construirGrafo(const Escenario& s, bool usarFiltro) {
    GrafoVisibilidad g;
    g.agregarNodo(s.base, TipoNodo::BASE, "D0");
    for (const Punto& p : s.estaciones) g.agregarNodo(p, TipoNodo::ESTACION, "S");
    for (const Punto& p : s.clientes) g.agregarNodo(p, TipoNodo::CLIENTE, "C");
    for (const Poligono& poligono : s.obstaculos) g.agregarObstaculo(poligono);
    g.construirAristas(usarFiltro);
    return g;
}

std::vector<int> vecinosOrdenados(const GrafoVisibilidad& g, int u) {
    std::vector<int> vecinosU;
    for (const Arista& e : g.vecinos(u)) vecinosU.push_back(e.destino);
    std::sort(vecinosU.begin(), vecinosU.end());
    return vecinosU;
}

void probarFiltroCajas(const std::vector<Escenario>& escenarios) {
    std::cout << "[PRUEBA] 1. El filtro por caja envolvente da las mismas aristas..." << std::endl;

    for (const Escenario& s : escenarios) {
        GrafoVisibilidad filtrado = construirGrafo(s, true);
        GrafoVisibilidad completo = construirGrafo(s, false);
        assert(filtrado.numAristas() == completo.numAristas());
        for (int u = 0; u < completo.numNodos(); ++u) {
            assert(vecinosOrdenados(filtrado, u) == vecinosOrdenados(completo, u));
        }
    }

    std::cout << "  -> OK\n" << std::endl;
}

void probarNodosClaveComoExtremos(const std::vector<Escenario>& escenarios) {
    std::cout << "[PRUEBA] 2. Dijkstra sin expandir nodos clave da las mismas distancias..." << std::endl;

    for (const Escenario& s : escenarios) {
        GrafoMision mision(s);
        const GrafoVisibilidad& g = mision.grafo();
        for (int a = 0; a < mision.numNodosClave(); ++a) {
            ResultadoDijkstra sinFiltro = Dijkstra::ejecutar(g, a, false);
            ResultadoDijkstra extremos = Dijkstra::ejecutar(g, a, true);
            for (int v = 0; v < g.numNodos(); ++v) {
                bool ambosInfinitos = std::isinf(sinFiltro.distancia[v]) && std::isinf(extremos.distancia[v]);
                assert(ambosInfinitos || std::abs(sinFiltro.distancia[v] - extremos.distancia[v]) < TOLERANCIA);
            }
        }
    }

    std::cout << "  -> OK\n" << std::endl;
}

void probarPropiedadesMatriz(const std::vector<Escenario>& escenarios) {
    std::cout << "[PRUEBA] 3. Matriz simetrica, diagonal cero y desigualdad triangular..." << std::endl;

    for (const Escenario& s : escenarios) {
        GrafoMision mision(s);
        int k = mision.numNodosClave();
        assert(k == 1 + s.numEstaciones() + s.numClientes());
        for (int a = 0; a < k; ++a) {
            assert(mision.distancia(a, a) == 0.0);
            for (int b = 0; b < k; ++b) {
                double ab = mision.distancia(a, b);
                double ba = mision.distancia(b, a);
                assert((std::isinf(ab) && std::isinf(ba)) || std::abs(ab - ba) < TOLERANCIA);
                // Nunca más corto que la línea recta.
                assert(ab >= mision.puntoClave(a).distancia(mision.puntoClave(b)) - TOLERANCIA);
                for (int c = 0; c < k; ++c) {
                    double porB = ab + mision.distancia(b, c);
                    assert(mision.distancia(a, c) <= porB + TOLERANCIA);
                }
            }
        }
    }

    std::cout << "  -> OK\n" << std::endl;
}

void probarCaminos(const std::vector<Escenario>& escenarios) {
    std::cout << "[PRUEBA] 4. Caminos reales: validos, solo doblan en obstaculos y miden lo de la matriz..."
              << std::endl;

    for (const Escenario& s : escenarios) {
        GrafoMision mision(s);
        const GrafoVisibilidad& g = mision.grafo();
        for (int a = 0; a < mision.numNodosClave(); ++a) {
            for (int b = 0; b < mision.numNodosClave(); ++b) {
                std::vector<int> camino = mision.nodosCamino(a, b);
                if (std::isinf(mision.distancia(a, b))) {
                    assert(camino.empty());
                    continue;
                }
                assert(camino.front() == a && camino.back() == b);
                double longitud = 0.0;
                for (size_t i = 0; i + 1 < camino.size(); ++i) {
                    if (i > 0) assert(g.nodo(camino[i]).tipo == TipoNodo::VERTICE_OBSTACULO);
                    Segmento tramo(g.nodo(camino[i]).punto, g.nodo(camino[i + 1]).punto);
                    assert(Geometria::tramoLibre(tramo, s.obstaculos));
                    longitud += tramo.longitud();
                }
                assert(std::abs(longitud - mision.distancia(a, b)) < TOLERANCIA);
            }
        }
    }

    std::cout << "  -> OK\n" << std::endl;
}

void probarDistanciaCalculadaAMano() {
    std::cout << "[PRUEBA] 5. Distancia calculada a mano alrededor de un cuadrado..." << std::endl;

    // Base (0,5) y cliente (10,5) con el cuadrado [3,7]x[3,7] en medio. El camino mínimo rodea
    // el cuadrado por abajo: (0,5) -> (3,3) -> (7,3) -> (10,5) = 2·sqrt(13) + 4.
    Escenario s;
    s.base = Punto(0, 5);
    s.clientes = {Punto(10, 5)};
    s.obstaculos = {Poligono(1, {Punto(3, 3), Punto(7, 3), Punto(7, 7), Punto(3, 7)})};
    GrafoMision mision(s);
    assert(std::abs(mision.distancia(0, mision.claveCliente(0)) - (2.0 * std::sqrt(13.0) + 4.0)) < TOLERANCIA);
    assert(mision.nodosCamino(0, mision.claveCliente(0)).size() == 4);

    std::cout << "  -> OK\n" << std::endl;
}

void probarCasosAdversariales(const std::string& carpeta) {
    std::cout << "[PRUEBA] 6. Casos adversariales: DFS, pasillo, borde y trampa concava..." << std::endl;

    // Cliente encerrado: el DFS desde la base no lo alcanza y no hay distancia finita.
    GrafoMision encerrado(ArchivoEscenario::cargar(carpeta + "/adv_enclosed.txt"));
    assert(encerrado.clientesInalcanzables() == std::vector<int>{0});
    assert(!encerrado.conectadoABase(encerrado.claveCliente(0)));
    assert(std::isinf(encerrado.distancia(0, encerrado.claveCliente(0))));
    assert(encerrado.conectadoABase(encerrado.claveCliente(1)));
    assert(encerrado.conectadoABase(encerrado.claveEstacion(0)));

    // Pasillo de ancho 2: la ruta va en línea recta de (10,50) a (90,50).
    GrafoMision pasillo(ArchivoEscenario::cargar(carpeta + "/adv_corridor.txt"));
    assert(std::abs(pasillo.distancia(0, pasillo.claveCliente(0)) - 80.0) < TOLERANCIA);
    assert(pasillo.clientesInalcanzables().empty());

    // Vuelo sobre la línea del borde inferior: de (10,30) a (90,30) en línea recta.
    GrafoMision colineal(ArchivoEscenario::cargar(carpeta + "/adv_collinear.txt"));
    assert(std::abs(colineal.distancia(0, colineal.claveCliente(0)) - 80.0) < TOLERANCIA);

    // Trampa en U: el cliente del hueco se alcanza recto; el de atrás obliga a rodear.
    GrafoMision trampa(ArchivoEscenario::cargar(carpeta + "/adv_concave_trap.txt"));
    assert(std::abs(trampa.distancia(0, trampa.claveCliente(0)) - 35.0) < TOLERANCIA);
    double detras = trampa.distancia(0, trampa.claveCliente(1));
    assert(!std::isinf(detras) && detras > 70.0 + 1.0);

    std::cout << "  -> OK\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string carpetaAdversarial = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DEL GRAFO DE MISION      " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    std::vector<Escenario> escenarios = escenariosDeMuestra(carpetaAdversarial);
    probarFiltroCajas(escenarios);
    probarNodosClaveComoExtremos(escenarios);
    probarPropiedadesMatriz(escenarios);
    probarCaminos(escenarios);
    probarDistanciaCalculadaAMano();
    probarCasosAdversariales(carpetaAdversarial);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DEL GRAFO PASARON CON EXITO         " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
