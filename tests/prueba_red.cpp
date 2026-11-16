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

const double TOLERANCE = 1e-6;

Escenario makeScenario(const Punto& base, const std::vector<Punto>& estaciones, const std::vector<Punto>& clientes) {
    Escenario s;
    s.base = base;
    s.estaciones = estaciones;
    s.clientes = clientes;
    return s;
}

Escenario layout(Categoria categoria, uint64_t semilla) {
    return GeneradorDatasets::generarGeometria(GeneradorDatasets::rangosDe(categoria), categoria, semilla);
}

// Simula la ruta tramo a tramo y comprueba todo lo que el juez exigirá:
// empieza y termina en la base, visita cada cliente una vez, la batería nunca baja de 0,
// al llegar a un cliente queda batería para su recarga más cercana, y costo y recargas cuadran.
void assertValidPlan(const MissionGraph& mission, const ChargingNetwork& network,
                     const std::vector<int>& order, const RechargePlan& plan) {
    assert(plan.feasible);
    const std::vector<int>& seq = plan.keySequence;
    assert(seq.front() == mission.depotKey() && seq.back() == mission.depotKey());

    std::vector<int> visited;
    double bateria = network.bateria();
    double cost = 0.0;
    int recharges = 0;
    for (size_t i = 1; i < seq.size(); ++i) {
        double d = mission.distance(seq[i - 1], seq[i]);
        bateria -= d;
        cost += d;
        assert(bateria >= -TOLERANCE);
        bool isRechargePoint = seq[i] <= mission.numEstaciones();
        if (isRechargePoint) {
            bateria = network.bateria();
            if (i + 1 < seq.size()) ++recharges;
        } else {
            visited.push_back(seq[i]);
            assert(bateria + TOLERANCE >= network.distanceToRecharge(seq[i]));
        }
    }
    assert(visited == order);   // Mismos clientes, mismo orden, sin repetir
    assert(std::abs(cost - plan.cost) < TOLERANCE);
    assert(recharges == plan.recharges);
}

void testPrim() {
    std::cout << "[TEST] 1. Prim sobre una matriz calculada a mano..." << std::endl;

    // Cuadrado de lado 1: el árbol usa tres lados (costo 3, arista mayor 1).
    const double r2 = std::sqrt(2.0);
    std::vector<std::vector<double>> dist = {
        {0, 1, r2, 1, INF},
        {1, 0, 1, r2, INF},
        {r2, 1, 0, 1, INF},
        {1, r2, 1, 0, INF},
        {INF, INF, INF, INF, 0},
    };
    MstResult square = primMst(dist, {0, 1, 2, 3});
    assert(std::abs(square.cost - 3.0) < TOLERANCE);
    assert(std::abs(square.maxEdge - 1.0) < TOLERANCE);
    assert(square.spansAll && square.parent[0] == -1);

    // Subconjunto de dos nodos opuestos: una sola arista, la diagonal.
    assert(std::abs(primMst(dist, {0, 2}).cost - r2) < TOLERANCE);

    // Un nodo sin camino deja el árbol incompleto.
    assert(!primMst(dist, {0, 1, 4}).spansAll);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testNetworkStructure() {
    std::cout << "[TEST] 2. Red de recarga: alcanzables, articulacion, dRecarga y red minima..." << std::endl;

    // Base y estaciones en cadena: D(0,0) - S1(10,0) - S2(20,0) - S3(30,0), y S4(20,8) colgando
    // de S2. Con B = 10.5 solo existen esas aristas, así que S1 y S2 son críticas.
    Escenario s = makeScenario(Punto(0, 0),
                              {Punto(10, 0), Punto(20, 0), Punto(30, 0), Punto(20, 8)},
                              {Punto(0, 6), Punto(30, 4)});
    MissionGraph mission(s);
    ChargingNetwork network(mission, 10.5);

    assert(network.linked(0, 1) && network.linked(2, 4) && !network.linked(1, 4) && !network.linked(0, 2));
    assert(network.reachablePoints() == std::vector<int>({0, 1, 2, 3, 4}));
    assert(network.criticalStations() == std::vector<int>({1, 2}));

    // Cliente (0,6): su recarga más cercana es la base, a 6. Ir y volver exige 12 > 10.5.
    int farClient = mission.clientKey(0);
    assert(std::abs(network.distanceToRecharge(farClient) - 6.0) < TOLERANCE);
    assert(!network.canServe(farClient));
    // Cliente (30,4): S3 a 4. Ir y volver exige 8 ≤ 10.5.
    assert(network.canServe(mission.clientKey(1)));
    assert(!ChargingNetwork::isFeasible(mission, 10.5));

    // Con B < 10 la base no llega a ninguna estación.
    ChargingNetwork weak(mission, 9.99);
    assert(weak.reachablePoints() == std::vector<int>({0}));
    assert(weak.criticalStations().empty());

    // Red mínima de soporte: aristas 10, 10, 10 y 8.
    std::vector<int> nodes;
    MstResult tree = ChargingNetwork::supportTree(mission, nodes);
    assert(nodes.size() == 5 && tree.spansAll);
    assert(std::abs(tree.cost - 38.0) < TOLERANCE);
    assert(std::abs(tree.maxEdge - 10.0) < TOLERANCE);

    // La arista mayor del árbol es la menor autonomía que conecta todas las estaciones.
    assert(ChargingNetwork(mission, tree.maxEdge).reachablePoints().size() == 5);
    assert(ChargingNetwork(mission, tree.maxEdge - 0.01).reachablePoints().size() < 5);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testMinimumAutonomyByHand() {
    std::cout << "[TEST] 3. Autonomia minima calculada a mano..." << std::endl;

    // Base (0,0), estación (10,0), cliente (10,6).
    // Con B ≥ 10 la estación es alcanzable y el cliente queda a 6 de ella: hace falta B ≥ 12.
    // Con B < 10 solo sirve la base, a sqrt(136) ≈ 11.66: haría falta B ≥ 23.3, contradicción.
    Escenario s = makeScenario(Punto(0, 0), {Punto(10, 0)}, {Punto(10, 6)});
    MissionGraph mission(s);
    assert(std::abs(ChargingNetwork::minimumAutonomy(mission) - 12.0) < TOLERANCE);
    assert(ChargingNetwork::isFeasible(mission, 12.0));
    assert(!ChargingNetwork::isFeasible(mission, 11.99));

    std::cout << "  -> PASSED\n" << std::endl;
}

void testMinimumAutonomyIsMonotone() {
    std::cout << "[TEST] 4. Busqueda binaria: factibilidad monotona en escenarios generados..." << std::endl;

    for (Categoria categoria : {Categoria::SMALL, Categoria::MEDIUM}) {
        for (uint64_t semilla = 1; semilla <= 3; ++semilla) {
            MissionGraph mission(layout(categoria, semilla));
            double minimum = ChargingNetwork::minimumAutonomy(mission);
            assert(ChargingNetwork::isFeasible(mission, minimum));
            assert(!ChargingNetwork::isFeasible(mission, minimum * 0.999));

            // Recorriendo B de menor a mayor, una vez factible nunca deja de serlo.
            bool seenFeasible = false;
            for (int step = 0; step <= 40; ++step) {
                bool feasible = ChargingNetwork::isFeasible(mission, minimum * 2.0 * step / 40.0);
                assert(!(seenFeasible && !feasible));
                seenFeasible = seenFeasible || feasible;
            }
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testPlannerByHand() {
    std::cout << "[TEST] 5. Insercion de recargas calculada a mano..." << std::endl;

    // Mismo escenario que la prueba 3 con B = 12. Desde la base, ir directo al cliente (11.66)
    // dejaría 0.34 y su recarga más cercana está a 6: se desvía por la estación. De vuelta,
    // con 6 de batería no llega a la base (11.66): vuelve a pasar por la estación.
    // Ruta: D -> S1 -> C1 -> S1 -> D = 10 + 6 + 6 + 10 = 32, con 2 recargas.
    Escenario s = makeScenario(Punto(0, 0), {Punto(10, 0)}, {Punto(10, 6)});
    MissionGraph mission(s);
    ChargingNetwork network(mission, 12.0);
    RechargePlanner planner(mission, network);
    std::vector<int> order = {mission.clientKey(0)};
    RechargePlan plan = planner.plan(order);

    assert(plan.keySequence == std::vector<int>({0, 1, 2, 1, 0}));
    assert(plan.recharges == 2);
    assert(std::abs(plan.cost - 32.0) < TOLERANCE);
    assertValidPlan(mission, network, order, plan);

    // Con batería de sobra no hace falta ninguna recarga.
    ChargingNetwork large(mission, 100.0);
    RechargePlan direct = RechargePlanner(mission, large).plan(order);
    assert(direct.keySequence == std::vector<int>({0, 2, 0}) && direct.recharges == 0);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testPlannerOnGenerated() {
    std::cout << "[TEST] 6. Rutas con recargas validas en escenarios generados (B minima y B x 1.2)..." << std::endl;

    std::mt19937 rng(2026);
    for (Categoria categoria : {Categoria::SMALL, Categoria::MEDIUM, Categoria::LARGE}) {
        for (uint64_t semilla = 1; semilla <= 2; ++semilla) {
            MissionGraph mission(layout(categoria, semilla));
            assert(mission.unreachableClients().empty());   // El generador nunca encierra clientes

            double minimum = ChargingNetwork::minimumAutonomy(mission);
            for (double bateria : {minimum, std::ceil(minimum * 1.2)}) {
                ChargingNetwork network(mission, bateria);
                RechargePlanner planner(mission, network);

                std::vector<int> order;
                for (int c = 0; c < mission.numClientes(); ++c) order.push_back(mission.clientKey(c));
                for (int attempt = 0; attempt < 3; ++attempt) {
                    std::shuffle(order.begin(), order.end(), rng);
                    assertValidPlan(mission, network, order, planner.plan(order));
                }
            }
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testAdversarialBattery(const std::string& folder) {
    std::cout << "[TEST] 7. Casos adversariales: cliente fuera de alcance y cliente encerrado..." << std::endl;

    // Con B = 60 el cliente (95,95) queda lejos de toda recarga; los otros tres se atienden.
    Escenario s = ArchivoEscenario::cargar(folder + "/adv_unreachable_battery.txt");
    MissionGraph mission(s);
    ChargingNetwork network(mission, s.bateria);
    assert(!network.canServe(mission.clientKey(3)));
    for (int c = 0; c < 3; ++c) assert(network.canServe(mission.clientKey(c)));
    assert(!ChargingNetwork::isFeasible(mission, s.bateria));

    std::vector<int> servable = {mission.clientKey(0), mission.clientKey(1), mission.clientKey(2)};
    assertValidPlan(mission, network, servable, RechargePlanner(mission, network).plan(servable));

    // El cliente encerrado no cuenta para la autonomía mínima: la separan obstáculos, no batería.
    MissionGraph enclosed(ArchivoEscenario::cargar(folder + "/adv_enclosed.txt"));
    double minimum = ChargingNetwork::minimumAutonomy(enclosed);
    assert(std::isfinite(minimum) && ChargingNetwork::isFeasible(enclosed, minimum));

    std::cout << "  -> PASSED\n" << std::endl;
}

void testGeneratorBattery() {
    std::cout << "[TEST] 8. Bateria del generador = autonomia minima x 1.2, redondeada..." << std::endl;

    for (uint64_t semilla = 1; semilla <= 3; ++semilla) {
        Escenario s = GeneradorDatasets::generar(Categoria::SMALL, semilla);
        MissionGraph mission(s);
        double expected = std::max(1.0, std::ceil(ChargingNetwork::minimumAutonomy(mission) * 1.2));
        assert(s.bateria == expected);
        assert(ChargingNetwork::isFeasible(mission, s.bateria));
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string adversarialFolder = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DE LA RED DE RECARGA     " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    testPrim();
    testNetworkStructure();
    testMinimumAutonomyByHand();
    testMinimumAutonomyIsMonotone();
    testPlannerByHand();
    testPlannerOnGenerated();
    testAdversarialBattery(adversarialFolder);
    testGeneratorBattery();

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DE LA RED DE RECARGA PASARON        " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
