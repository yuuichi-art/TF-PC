// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/grafo/RedRecarga.hpp"
#include "../src/grafo/GrafoMision.hpp"
#include "../src/grafo/ArbolMinimo.hpp"
#include "../src/solvers/PlanificadorRecargas.hpp"
#include "../src/escenario/GeneradorDatasets.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

using namespace geodrone;

const double TOLERANCIA = 1e-6;

Escenario crearEscenario(const Punto& base, const std::vector<Punto>& estaciones, const std::vector<Punto>& clientes) {
    Escenario s;
    s.base = base;
    s.estaciones = estaciones;
    s.clientes = clientes;
    return s;
}

Escenario soloGeometria(Categoria categoria, uint64_t semilla) {
    return GeneradorDatasets::generarGeometria(GeneradorDatasets::rangosDe(categoria), categoria, semilla);
}

// Simula la ruta tramo a tramo y comprueba todo lo que el juez exigirá:
// empieza y termina en la base, visita cada cliente una vez, la batería nunca baja de 0,
// al llegar a un cliente queda batería para su recarga más cercana, y costo y recargas cuadran.
void verificarPlanValido(const GrafoMision& mision, const RedRecarga& red,
                     const std::vector<int>& order, const RechargePlan& plan) {
    assert(plan.feasible);
    const std::vector<int>& secuencia = plan.keySequence;
    assert(secuencia.front() == mision.claveBase() && secuencia.back() == mision.claveBase());

    std::vector<int> visitados;
    double bateria = red.bateria();
    double costo = 0.0;
    int recharges = 0;
    for (size_t i = 1; i < secuencia.size(); ++i) {
        double d = mision.distancia(secuencia[i - 1], secuencia[i]);
        bateria -= d;
        costo += d;
        assert(bateria >= -TOLERANCIA);
        bool esPuntoRecarga = secuencia[i] <= mision.numEstaciones();
        if (esPuntoRecarga) {
            bateria = red.bateria();
            if (i + 1 < secuencia.size()) ++recharges;
        } else {
            visitados.push_back(secuencia[i]);
            assert(bateria + TOLERANCIA >= red.distanciaARecarga(secuencia[i]));
        }
    }
    assert(visitados == order);   // Mismos clientes, mismo orden, sin repetir
    assert(std::abs(costo - plan.costo) < TOLERANCIA);
    assert(recharges == plan.recharges);
}

void probarPrim() {
    std::cout << "[TEST] 1. Prim sobre una matriz calculada a mano..." << std::endl;

    // Cuadrado de lado 1: el árbol usa tres lados (costo 3, arista mayor 1).
    const double r2 = std::sqrt(2.0);
    std::vector<std::vector<double>> distancias = {
        {0, 1, r2, 1, INF},
        {1, 0, 1, r2, INF},
        {r2, 1, 0, 1, INF},
        {1, r2, 1, 0, INF},
        {INF, INF, INF, INF, 0},
    };
    ArbolMinimo cuadrado = arbolMinimoPrim(distancias, {0, 1, 2, 3});
    assert(std::abs(cuadrado.costo - 3.0) < TOLERANCIA);
    assert(std::abs(cuadrado.aristaMaxima - 1.0) < TOLERANCIA);
    assert(cuadrado.cubreTodos && cuadrado.padre[0] == -1);

    // Subconjunto de dos nodos opuestos: una sola arista, la diagonal.
    assert(std::abs(arbolMinimoPrim(distancias, {0, 2}).costo - r2) < TOLERANCIA);

    // Un nodo sin camino deja el árbol incompleto.
    assert(!arbolMinimoPrim(distancias, {0, 1, 4}).cubreTodos);

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarEstructuraRed() {
    std::cout << "[TEST] 2. Red de recarga: alcanzables, articulacion, dRecarga y red minima..." << std::endl;

    // Base y estaciones en cadena: D(0,0) - S1(10,0) - S2(20,0) - S3(30,0), y S4(20,8) colgando
    // de S2. Con B = 10.5 solo existen esas aristas, así que S1 y S2 son críticas.
    Escenario s = crearEscenario(Punto(0, 0),
                              {Punto(10, 0), Punto(20, 0), Punto(30, 0), Punto(20, 8)},
                              {Punto(0, 6), Punto(30, 4)});
    GrafoMision mision(s);
    RedRecarga red(mision, 10.5);

    assert(red.conectados(0, 1) && red.conectados(2, 4) && !red.conectados(1, 4) && !red.conectados(0, 2));
    assert(red.puntosAlcanzables() == std::vector<int>({0, 1, 2, 3, 4}));
    assert(red.estacionesCriticas() == std::vector<int>({1, 2}));

    // Cliente (0,6): su recarga más cercana es la base, a 6. Ir y volver exige 12 > 10.5.
    int clienteLejano = mision.claveCliente(0);
    assert(std::abs(red.distanciaARecarga(clienteLejano) - 6.0) < TOLERANCIA);
    assert(!red.puedeAtender(clienteLejano));
    // Cliente (30,4): S3 a 4. Ir y volver exige 8 ≤ 10.5.
    assert(red.puedeAtender(mision.claveCliente(1)));
    assert(!RedRecarga::esFactible(mision, 10.5));

    // Con B < 10 la base no llega a ninguna estación.
    RedRecarga debil(mision, 9.99);
    assert(debil.puntosAlcanzables() == std::vector<int>({0}));
    assert(debil.estacionesCriticas().empty());

    // Red mínima de soporte: aristas 10, 10, 10 y 8.
    std::vector<int> nodos;
    ArbolMinimo arbol = RedRecarga::arbolSoporte(mision, nodos);
    assert(nodos.size() == 5 && arbol.cubreTodos);
    assert(std::abs(arbol.costo - 38.0) < TOLERANCIA);
    assert(std::abs(arbol.aristaMaxima - 10.0) < TOLERANCIA);

    // La arista mayor del árbol es la menor autonomía que conecta todas las estaciones.
    assert(RedRecarga(mision, arbol.aristaMaxima).puntosAlcanzables().size() == 5);
    assert(RedRecarga(mision, arbol.aristaMaxima - 0.01).puntosAlcanzables().size() < 5);

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarAutonomiaMinimaAMano() {
    std::cout << "[TEST] 3. Autonomia minima calculada a mano..." << std::endl;

    // Base (0,0), estación (10,0), cliente (10,6).
    // Con B ≥ 10 la estación es alcanzable y el cliente queda a 6 de ella: hace falta B ≥ 12.
    // Con B < 10 solo sirve la base, a sqrt(136) ≈ 11.66: haría falta B ≥ 23.3, contradicción.
    Escenario s = crearEscenario(Punto(0, 0), {Punto(10, 0)}, {Punto(10, 6)});
    GrafoMision mision(s);
    assert(std::abs(RedRecarga::autonomiaMinima(mision) - 12.0) < TOLERANCIA);
    assert(RedRecarga::esFactible(mision, 12.0));
    assert(!RedRecarga::esFactible(mision, 11.99));

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarAutonomiaMinimaMonotona() {
    std::cout << "[TEST] 4. Busqueda binaria: factibilidad monotona en escenarios generados..." << std::endl;

    for (Categoria categoria : {Categoria::SMALL, Categoria::MEDIUM}) {
        for (uint64_t semilla = 1; semilla <= 3; ++semilla) {
            GrafoMision mision(soloGeometria(categoria, semilla));
            double minima = RedRecarga::autonomiaMinima(mision);
            assert(RedRecarga::esFactible(mision, minima));
            assert(!RedRecarga::esFactible(mision, minima * 0.999));

            // Recorriendo B de menor a mayor, una vez factible nunca deja de serlo.
            bool vioFactible = false;
            for (int paso = 0; paso <= 40; ++paso) {
                bool feasible = RedRecarga::esFactible(mision, minima * 2.0 * paso / 40.0);
                assert(!(vioFactible && !feasible));
                vioFactible = vioFactible || feasible;
            }
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarPlanificadorAMano() {
    std::cout << "[TEST] 5. Insercion de recargas calculada a mano..." << std::endl;

    // Mismo escenario que la prueba 3 con B = 12. Desde la base, ir directo al cliente (11.66)
    // dejaría 0.34 y su recarga más cercana está a 6: se desvía por la estación. De vuelta,
    // con 6 de batería no llega a la base (11.66): vuelve a pasar por la estación.
    // Ruta: D -> S1 -> C1 -> S1 -> D = 10 + 6 + 6 + 10 = 32, con 2 recargas.
    Escenario s = crearEscenario(Punto(0, 0), {Punto(10, 0)}, {Punto(10, 6)});
    GrafoMision mision(s);
    RedRecarga red(mision, 12.0);
    RechargePlanner planner(mision, red);
    std::vector<int> order = {mision.claveCliente(0)};
    RechargePlan plan = planner.plan(order);

    assert(plan.keySequence == std::vector<int>({0, 1, 2, 1, 0}));
    assert(plan.recharges == 2);
    assert(std::abs(plan.costo - 32.0) < TOLERANCIA);
    verificarPlanValido(mision, red, order, plan);

    // Con batería de sobra no hace falta ninguna recarga.
    RedRecarga grande(mision, 100.0);
    RechargePlan directo = RechargePlanner(mision, grande).plan(order);
    assert(directo.keySequence == std::vector<int>({0, 2, 0}) && directo.recharges == 0);

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarPlanificadorEnGenerados() {
    std::cout << "[TEST] 6. Rutas con recargas validas en escenarios generados (B minima y B x 1.2)..." << std::endl;

    std::mt19937 rng(2026);
    for (Categoria categoria : {Categoria::SMALL, Categoria::MEDIUM, Categoria::LARGE}) {
        for (uint64_t semilla = 1; semilla <= 2; ++semilla) {
            GrafoMision mision(soloGeometria(categoria, semilla));
            assert(mision.clientesInalcanzables().empty());   // El generador nunca encierra clientes

            double minima = RedRecarga::autonomiaMinima(mision);
            for (double bateria : {minima, std::ceil(minima * 1.2)}) {
                RedRecarga red(mision, bateria);
                RechargePlanner planner(mision, red);

                std::vector<int> order;
                for (int c = 0; c < mision.numClientes(); ++c) order.push_back(mision.claveCliente(c));
                for (int intento = 0; intento < 3; ++intento) {
                    std::shuffle(order.begin(), order.end(), rng);
                    verificarPlanValido(mision, red, order, planner.plan(order));
                }
            }
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarBateriaAdversarial(const std::string& carpeta) {
    std::cout << "[TEST] 7. Casos adversariales: cliente fuera de alcance y cliente encerrado..." << std::endl;

    // Con B = 60 el cliente (95,95) queda lejos de toda recarga; los otros tres se atienden.
    Escenario s = ArchivoEscenario::cargar(carpeta + "/adv_unreachable_battery.txt");
    GrafoMision mision(s);
    RedRecarga red(mision, s.bateria);
    assert(!red.puedeAtender(mision.claveCliente(3)));
    for (int c = 0; c < 3; ++c) assert(red.puedeAtender(mision.claveCliente(c)));
    assert(!RedRecarga::esFactible(mision, s.bateria));

    std::vector<int> atendibles = {mision.claveCliente(0), mision.claveCliente(1), mision.claveCliente(2)};
    verificarPlanValido(mision, red, atendibles, RechargePlanner(mision, red).plan(atendibles));

    // El cliente encerrado no cuenta para la autonomía mínima: la separan obstáculos, no batería.
    GrafoMision encerrado(ArchivoEscenario::cargar(carpeta + "/adv_enclosed.txt"));
    double minima = RedRecarga::autonomiaMinima(encerrado);
    assert(std::isfinite(minima) && RedRecarga::esFactible(encerrado, minima));

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarBateriaGenerador() {
    std::cout << "[TEST] 8. Bateria del generador = autonomia minima x 1.2, redondeada..." << std::endl;

    for (uint64_t semilla = 1; semilla <= 3; ++semilla) {
        Escenario s = GeneradorDatasets::generar(Categoria::SMALL, semilla);
        GrafoMision mision(s);
        double esperado = std::max(1.0, std::ceil(RedRecarga::autonomiaMinima(mision) * 1.2));
        assert(s.bateria == esperado);
        assert(RedRecarga::esFactible(mision, s.bateria));
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string carpetaAdversarial = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DE LA RED DE RECARGA     " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    probarPrim();
    probarEstructuraRed();
    probarAutonomiaMinimaAMano();
    probarAutonomiaMinimaMonotona();
    probarPlanificadorAMano();
    probarPlanificadorEnGenerados();
    probarBateriaAdversarial(carpetaAdversarial);
    probarBateriaGenerador();

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DE LA RED DE RECARGA PASARON        " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
