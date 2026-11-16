// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/solvers/SolverExacto.hpp"
#include "../src/solvers/SolverHeuristico.hpp"
#include "../src/grafo/GrafoMision.hpp"
#include "../src/grafo/ArbolMinimo.hpp"
#include "../src/escenario/GeneradorDatasets.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;
using Matriz = std::vector<std::vector<double>>;

const double TOLERANCIA = 1e-6;

// Costo del ciclo base -> clientes en ese orden -> base.
double costoCicloAMano(const Matriz& distancias, int base, const std::vector<int>& orden) {
    double costo = 0.0;
    int posicion = base;
    for (int cliente : orden) {
        costo += distancias[posicion][cliente];
        posicion = cliente;
    }
    return costo + distancias[posicion][base];
}

// Fuerza bruta: prueba todos los órdenes. O(n! · n); solo para n pequeño.
double fuerzaBruta(const Matriz& distancias, int base, std::vector<int> clientes) {
    std::sort(clientes.begin(), clientes.end());
    double mejor = INF;
    do {
        mejor = std::min(mejor, costoCicloAMano(distancias, base, clientes));
    } while (std::next_permutation(clientes.begin(), clientes.end()));
    return mejor;
}

bool esPermutacionDe(std::vector<int> orden, std::vector<int> clientes) {
    std::sort(orden.begin(), orden.end());
    std::sort(clientes.begin(), clientes.end());
    return orden == clientes;
}

Matriz matrizEuclidiana(const std::vector<Punto>& puntos) {
    Matriz distancias(puntos.size(), std::vector<double>(puntos.size()));
    for (size_t i = 0; i < puntos.size(); ++i) {
        for (size_t j = 0; j < puntos.size(); ++j) distancias[i][j] = puntos[i].distancia(puntos[j]);
    }
    return distancias;
}

// El exacto devuelve una permutación de los clientes, su costo es el informado y coincide
// con el de la fuerza bruta.
ResultadoExacto verificarOptimo(const Matriz& distancias, int base, const std::vector<int>& clientes) {
    ResultadoExacto resultado = SolverExacto::resolver(distancias, base, clientes);
    assert(esPermutacionDe(resultado.orden, clientes));
    assert(std::abs(costoCicloAMano(distancias, base, resultado.orden) - resultado.costo) < TOLERANCIA);
    assert(std::abs(resultado.costo - fuerzaBruta(distancias, base, clientes)) < TOLERANCIA);
    return resultado;
}

void probarEjemplosAMano() {
    std::cout << "[TEST] 1. Casos calculados a mano..." << std::endl;

    // Cuadrado de lado 10 con la base en una esquina: el óptimo es el perímetro, 40.
    Matriz cuadrado = matrizEuclidiana({Punto(0, 0), Punto(10, 0), Punto(10, 10), Punto(0, 10)});
    assert(std::abs(verificarOptimo(cuadrado, 0, {1, 2, 3}).costo - 40.0) < TOLERANCIA);

    // Puntos en una recta: base en 0 y clientes en 1, -2 y 4. El vecino más cercano haría
    // 0 -> 1 -> -2 -> 4 -> 0 = 14; el óptimo es 12 (por ejemplo 0 -> 1 -> 4 -> -2 -> 0).
    Matriz linea = matrizEuclidiana({Punto(0, 0), Punto(1, 0), Punto(-2, 0), Punto(4, 0)});
    assert(std::abs(costoCicloAMano(linea, 0, {1, 2, 3}) - 14.0) < TOLERANCIA);
    assert(std::abs(verificarOptimo(linea, 0, {1, 2, 3}).costo - 12.0) < TOLERANCIA);

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarCasosBorde() {
    std::cout << "[TEST] 2. Casos borde: sin clientes, un cliente y demasiados clientes..." << std::endl;

    Matriz distancias = matrizEuclidiana({Punto(0, 0), Punto(3, 4)});
    ResultadoExacto vacio = SolverExacto::resolver(distancias, 0, {});
    assert(vacio.orden.empty() && vacio.costo == 0.0);

    ResultadoExacto unico = SolverExacto::resolver(distancias, 0, {1});
    assert(unico.orden == std::vector<int>{1} && std::abs(unico.costo - 10.0) < TOLERANCIA);

    std::vector<int> demasiados(SolverExacto::MAX_CLIENTES + 1, 1);
    bool rechazado = false;
    try {
        SolverExacto::resolver(distancias, 0, demasiados);
    } catch (const std::invalid_argument&) {
        rechazado = true;
    }
    assert(rechazado);

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarAleatoriasContraFuerzaBruta() {
    std::cout << "[TEST] 3. Instancias aleatorias de 1 a 8 clientes contra fuerza bruta..." << std::endl;

    std::mt19937 rng(2026);
    std::uniform_int_distribution<int> coordenada(0, 100);
    for (int n = 1; n <= 8; ++n) {
        for (int instancia = 0; instancia < 10; ++instancia) {
            std::vector<Punto> puntos;
            for (int i = 0; i <= n; ++i) puntos.emplace_back(coordenada(rng), coordenada(rng));
            std::vector<int> clientes;
            for (int i = 1; i <= n; ++i) clientes.push_back(i);
            verificarOptimo(matrizEuclidiana(puntos), 0, clientes);
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarEscenariosConObstaculos(const std::string& carpetaAdversarial) {
    std::cout << "[TEST] 4. Escenarios con obstaculos (generados y adversariales) contra fuerza bruta..." << std::endl;

    std::vector<Escenario> escenarios;
    for (uint64_t semilla = 1; semilla <= 15; ++semilla) {
        escenarios.push_back(GeneradorDatasets::generarGeometria(
            GeneradorDatasets::rangosDe(Categoria::SMALL), Categoria::SMALL, semilla));
    }
    for (const char* nombre : {"adv_collinear", "adv_enclosed", "adv_corridor", "adv_concave_trap"}) {
        escenarios.push_back(ArchivoEscenario::cargar(carpetaAdversarial + "/" + nombre + ".txt"));
    }

    for (const Escenario& s : escenarios) {
        GrafoMision mision(s);
        std::vector<int> inalcanzable = mision.clientesInalcanzables();
        std::vector<int> clientes;
        for (int c = 0; c < mision.numClientes() && clientes.size() < 8; ++c) {
            if (std::find(inalcanzable.begin(), inalcanzable.end(), c) == inalcanzable.end()) {
                clientes.push_back(mision.claveCliente(c));
            }
        }
        verificarOptimo(mision.distancias(), mision.claveBase(), clientes);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarPoda() {
    std::cout << "[TEST] 5. La poda evita recorrer todos los ordenes..." << std::endl;

    // Con 10 clientes hay 10! = 3 628 800 órdenes completos; el árbol entero tiene aún más nodos.
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> coordenada(0, 1000);
    for (int instancia = 0; instancia < 5; ++instancia) {
        std::vector<Punto> puntos;
        for (int i = 0; i <= 10; ++i) puntos.emplace_back(coordenada(rng), coordenada(rng));
        std::vector<int> clientes;
        for (int i = 1; i <= 10; ++i) clientes.push_back(i);
        ResultadoExacto resultado = SolverExacto::resolver(matrizEuclidiana(puntos), 0, clientes);
        std::cout << "  10 clientes: " << resultado.nodosExplorados << " nodos visitados, "
                  << resultado.nodosPodados << " podas" << std::endl;
        assert(resultado.nodosExplorados < 3628800 / 10);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

// ---------------------------------------------------------------------------------------------
// Heurísticas
// ---------------------------------------------------------------------------------------------

struct Instancia {
    Matriz distancias;
    int base;
    std::vector<int> clientes;
};

Instancia instanciaAleatoria(int n, std::mt19937& rng) {
    std::uniform_int_distribution<int> coordenada(0, 1000);
    std::vector<Punto> puntos;
    for (int i = 0; i <= n; ++i) puntos.emplace_back(coordenada(rng), coordenada(rng));
    Instancia instancia{matrizEuclidiana(puntos), 0, {}};
    for (int i = 1; i <= n; ++i) instancia.clientes.push_back(i);
    return instancia;
}

// Instancia con obstáculos a partir de un escenario generado (solo geometría).
Instancia instanciaDeEscenario(Categoria categoria, uint64_t semilla) {
    GrafoMision mision(GeneradorDatasets::generarGeometria(GeneradorDatasets::rangosDe(categoria), categoria, semilla));
    Instancia instancia{mision.distancias(), mision.claveBase(), {}};
    for (int c = 0; c < mision.numClientes(); ++c) instancia.clientes.push_back(mision.claveCliente(c));
    return instancia;
}

// Ningún intercambio 2-opt acorta la ruta: es un óptimo local.
bool esOptimoLocal2Opt(const Matriz& distancias, int base, const std::vector<int>& orden) {
    std::vector<int> ciclo = {base};
    ciclo.insert(ciclo.end(), orden.begin(), orden.end());
    ciclo.push_back(base);
    int n = static_cast<int>(orden.size());
    for (int i = 1; i < n; ++i) {
        for (int j = i + 1; j <= n; ++j) {
            double delta = distancias[ciclo[i - 1]][ciclo[j]] + distancias[ciclo[i]][ciclo[j + 1]]
                         - distancias[ciclo[i - 1]][ciclo[i]] - distancias[ciclo[j]][ciclo[j + 1]];
            if (delta < -1e-6) return false;
        }
    }
    return true;
}

void probarHeuristicasAMano() {
    std::cout << "[TEST] 6. Heuristicas en los casos calculados a mano..." << std::endl;

    // Recta: base en 0, clientes en 1, -2 y 4. El vecino más cercano hace 0 -> 1 -> -2 -> 4 -> 0 = 14
    // y 2-opt lo corrige hasta el óptimo, 12.
    Matriz linea = matrizEuclidiana({Punto(0, 0), Punto(1, 0), Punto(-2, 0), Punto(4, 0)});
    std::vector<int> voraz = SolverHeuristico::vecinoMasCercano(linea, 0, {1, 2, 3});
    assert(voraz == std::vector<int>({1, 2, 3}));
    assert(std::abs(SolverHeuristico::costoCiclo(linea, 0, voraz) - 14.0) < TOLERANCIA);
    assert(SolverHeuristico::aplicar2Opt(linea, 0, voraz));
    assert(std::abs(SolverHeuristico::costoCiclo(linea, 0, voraz) - 12.0) < TOLERANCIA);

    // Sin clientes, todas devuelven una ruta vacía.
    assert(SolverHeuristico::vecinoMasCercano(linea, 0, {}).empty());
    assert(SolverHeuristico::dosAproximacion(linea, 0, {}).empty());
    assert(SolverHeuristico::busquedaLocalIterada(linea, 0, {}, 1).orden.empty());

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarHeuristicasDanOrdenesValidos() {
    std::cout << "[TEST] 7. Cada heuristica devuelve una permutacion; 2-opt nunca empeora..." << std::endl;

    std::mt19937 rng(42);
    std::vector<Instancia> instancias;
    for (int n : {1, 2, 3, 4, 5, 20, 60}) instancias.push_back(instanciaAleatoria(n, rng));
    instancias.push_back(instanciaDeEscenario(Categoria::MEDIUM, 1));
    instancias.push_back(instanciaDeEscenario(Categoria::LARGE, 1));

    for (const Instancia& caso : instancias) {
        std::vector<int> voraz = SolverHeuristico::vecinoMasCercano(caso.distancias, caso.base, caso.clientes);
        std::vector<int> aprox = SolverHeuristico::dosAproximacion(caso.distancias, caso.base, caso.clientes);
        assert(esPermutacionDe(voraz, caso.clientes) && esPermutacionDe(aprox, caso.clientes));

        for (std::vector<int> inicio : {voraz, aprox}) {
            double antes = SolverHeuristico::costoCiclo(caso.distancias, caso.base, inicio);
            assert(SolverHeuristico::aplicar2Opt(caso.distancias, caso.base, inicio));
            assert(esPermutacionDe(inicio, caso.clientes));
            assert(SolverHeuristico::costoCiclo(caso.distancias, caso.base, inicio) <= antes + TOLERANCIA);
            assert(esOptimoLocal2Opt(caso.distancias, caso.base, inicio));
        }

        SolverHeuristico::ResultadoBusquedaIterada busqueda = SolverHeuristico::busquedaLocalIterada(caso.distancias, caso.base, voraz, 7, 30);
        assert(esPermutacionDe(busqueda.orden, caso.clientes) && !busqueda.cortadaPorTiempo);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarGarantiaAproximacion() {
    std::cout << "[TEST] 8. La 2-aproximacion cumple costo <= 2 x arbol minimo <= 2 x optimo..." << std::endl;

    std::mt19937 rng(9);
    for (int n = 1; n <= 9; ++n) {
        for (int instancia = 0; instancia < 5; ++instancia) {
            Instancia caso = instanciaAleatoria(n, rng);
            double aprox = SolverHeuristico::costoCiclo(caso.distancias, caso.base,
                                                      SolverHeuristico::dosAproximacion(caso.distancias, caso.base, caso.clientes));
            double optimo = SolverExacto::resolver(caso.distancias, caso.base, caso.clientes).costo;
            assert(aprox <= 2.0 * optimo + TOLERANCIA);
        }
    }

    // En instancias grandes no hay óptimo para comparar, pero sí el árbol mínimo, que es menor.
    for (uint64_t semilla = 1; semilla <= 3; ++semilla) {
        Instancia caso = instanciaDeEscenario(Categoria::LARGE, semilla);
        std::vector<int> nodos = {caso.base};
        nodos.insert(nodos.end(), caso.clientes.begin(), caso.clientes.end());
        double costoArbol = arbolMinimoPrim(caso.distancias, nodos).costo;
        double aprox = SolverHeuristico::costoCiclo(caso.distancias, caso.base,
                                                  SolverHeuristico::dosAproximacion(caso.distancias, caso.base, caso.clientes));
        assert(costoArbol <= aprox + TOLERANCIA && aprox <= 2.0 * costoArbol + TOLERANCIA);
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarBusquedaLocalIterada() {
    std::cout << "[TEST] 9. Busqueda local iterada: mejora, es reproducible y respeta el tiempo..." << std::endl;

    // Nunca peor que el 2-opt de su ruta inicial, y la misma semilla da la misma ruta.
    for (uint64_t semilla = 1; semilla <= 3; ++semilla) {
        Instancia caso = instanciaDeEscenario(Categoria::MEDIUM, semilla);
        std::vector<int> inicio = SolverHeuristico::vecinoMasCercano(caso.distancias, caso.base, caso.clientes);
        std::vector<int> optimoLocal = inicio;
        SolverHeuristico::aplicar2Opt(caso.distancias, caso.base, optimoLocal);

        SolverHeuristico::ResultadoBusquedaIterada primera = SolverHeuristico::busquedaLocalIterada(caso.distancias, caso.base, inicio, semilla);
        SolverHeuristico::ResultadoBusquedaIterada segunda = SolverHeuristico::busquedaLocalIterada(caso.distancias, caso.base, inicio, semilla);
        assert(primera.orden == segunda.orden);
        assert(primera.iteraciones == SolverHeuristico::ITERACIONES_POR_DEFECTO);
        assert(SolverHeuristico::costoCiclo(caso.distancias, caso.base, primera.orden) <=
               SolverHeuristico::costoCiclo(caso.distancias, caso.base, optimoLocal) + TOLERANCIA);
    }

    // En instancias Small llega al óptimo del branch and bound.
    for (uint64_t semilla = 1; semilla <= 10; ++semilla) {
        Instancia caso = instanciaDeEscenario(Categoria::SMALL, semilla);
        std::vector<int> inicio = SolverHeuristico::vecinoMasCercano(caso.distancias, caso.base, caso.clientes);
        double busqueda = SolverHeuristico::costoCiclo(caso.distancias, caso.base,
                                               SolverHeuristico::busquedaLocalIterada(caso.distancias, caso.base, inicio, semilla).orden);
        double optimo = SolverExacto::resolver(caso.distancias, caso.base, caso.clientes).costo;
        assert(busqueda <= optimo + TOLERANCIA);
    }

    // Con un límite de 30 ms y muchas iteraciones pedidas, se detiene a tiempo.
    Instancia grande = instanciaDeEscenario(Categoria::LARGE, 2);
    std::vector<int> inicio = SolverHeuristico::vecinoMasCercano(grande.distancias, grande.base, grande.clientes);
    auto comienzo = SolverHeuristico::Reloj::now();
    SolverHeuristico::ResultadoBusquedaIterada limitada = SolverHeuristico::busquedaLocalIterada(
        grande.distancias, grande.base, inicio, 1, 1000000, comienzo + std::chrono::milliseconds(30));
    double transcurridoMs = std::chrono::duration<double, std::milli>(SolverHeuristico::Reloj::now() - comienzo).count();
    assert(limitada.cortadaPorTiempo && esPermutacionDe(limitada.orden, grande.clientes));
    assert(transcurridoMs < 30.0 + 200.0);   // Margen para la última pasada de 2-opt y el sistema

    std::cout << "  -> PASSED\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string carpetaAdversarial = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DE LOS SOLVERS           " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    probarEjemplosAMano();
    probarCasosBorde();
    probarAleatoriasContraFuerzaBruta();
    probarEscenariosConObstaculos(carpetaAdversarial);
    probarPoda();
    probarHeuristicasAMano();
    probarHeuristicasDanOrdenesValidos();
    probarGarantiaAproximacion();
    probarBusquedaLocalIterada();

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DE LOS SOLVERS PASARON              " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
