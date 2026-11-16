// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/juez/Juez.hpp"
#include "../src/exportacion/Exportador.hpp"
#include "../src/escenario/GeneradorDatasets.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace geodrone;

Escenario handScenario() {
    // Base (0,0), estación (10,0), cliente (10,6), batería 12: D0 -> S1 -> C1 -> S1 -> D0.
    Escenario s;
    s.nombre = "hand";
    s.categoria = Categoria::ADVERSARIAL;
    s.bateria = 12;
    s.base = Punto(0, 0);
    s.estaciones = {Punto(10, 0)};
    s.clientes = {Punto(10, 6)};
    return s;
}

// La ruta debe rechazarse y el motivo debe mencionar el fragmento esperado.
void expectRejected(const Escenario& s, const SolverMision& solver, const Ruta& ruta, const std::string& fragment) {
    VerificationResult result = Judge::verifyRoute(s, solver, ruta);
    if (result.valid || result.problem.find(fragment) == std::string::npos) {
        std::cerr << "  Resultado: " << (result.valid ? "valida" : result.problem) << "\n  Se esperaba: " << fragment << "\n";
        assert(false);
    }
}

void removePoint(Ruta& ruta, size_t index) {
    ruta.trayectoria.erase(ruta.trayectoria.begin() + static_cast<long>(index));
}

void testAcceptsValidRoutes(const std::string& folder) {
    std::cout << "[TEST] 1. El verificador acepta las rutas de todos los algoritmos..." << std::endl;

    std::vector<Escenario> scenarios;
    for (const std::string& file : Judge::findScenarioFiles(folder)) scenarios.push_back(ArchivoEscenario::cargar(file));
    scenarios.push_back(GeneradorDatasets::generar(Categoria::SMALL, 1));
    scenarios.push_back(GeneradorDatasets::generar(Categoria::MEDIUM, 1));
    scenarios.push_back(handScenario());

    for (const Escenario& s : scenarios) {
        SolverMision solver(s);
        for (Algoritmo algoritmo : todosLosAlgoritmos()) {
            if (algoritmo == Algoritmo::BRANCH_AND_BOUND && !solver.admiteExacto()) continue;
            VerificationResult result = Judge::verifyRoute(s, solver, solver.resolver(algoritmo));
            if (!result.valid) std::cerr << "  " << s.nombre << ": " << result.problem << "\n";
            assert(result.valid);
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void testRejectsInvalidRoutes(const std::string& folder) {
    std::cout << "[TEST] 2. El verificador rechaza cada tipo de ruta invalida..." << std::endl;

    // Trampa en U: sin los vértices de obstáculo, la ruta atraviesa la U.
    Escenario trap = ArchivoEscenario::cargar(folder + "/adv_concave_trap.txt");
    SolverMision trapSolver(trap);
    Ruta straight = trapSolver.resolver(Algoritmo::BRANCH_AND_BOUND);
    straight.trayectoria.erase(std::remove_if(straight.trayectoria.begin(), straight.trayectoria.end(),
                                       [](const PuntoRuta& p) { return p.etiqueta == "V"; }),
                        straight.trayectoria.end());
    expectRejected(trap, trapSolver, straight, "cruza un obstaculo");

    // Ruta a mano D0 S1 C1 S1 D0, que es válida, y variantes rotas.
    Escenario hand = handScenario();
    SolverMision solver(hand);
    const Ruta good = solver.resolver(Algoritmo::BRANCH_AND_BOUND);
    assert(Judge::verifyRoute(hand, solver, good).valid);

    Escenario weak = hand;
    weak.bateria = 8;                                   // Con 8 no alcanza el tramo de 10 a la estación
    expectRejected(weak, solver, good, "bateria se agota");

    Ruta r = good;
    removePoint(r, 2);                                  // Sin C1
    expectRejected(hand, solver, r, "C1 no se visita ni esta excluido");

    r = good;
    r.trayectoria.insert(r.trayectoria.begin() + 2, r.trayectoria[2]);       // C1 dos veces
    expectRejected(hand, solver, r, "C1 se visita mas de una vez");

    r = good;
    r.trayectoria[2].punto.x += 1;                             // C1 fuera de su lugar
    expectRejected(hand, solver, r, "C1 no esta en las coordenadas");

    r = good;
    removePoint(r, 0);                                  // No empieza en la base
    expectRejected(hand, solver, r, "no empieza y termina en la base");

    r = good;
    r.costo += 5;
    expectRejected(hand, solver, r, "costo informado");

    r = good;
    r.recargas = 0;
    expectRejected(hand, solver, r, "recargas informadas");

    r = good;
    std::swap(r.orden[1], r.orden[2]);                  // ORDER no sigue a la trayectoria
    expectRejected(hand, solver, r, "orden informado");

    r = good;
    r.trayectoria[1].etiqueta = "X9";
    expectRejected(hand, solver, r, "etiqueta desconocida");

    r = good;
    r.factible = false;
    expectRejected(hand, solver, r, "no encontro una ruta factible");

    // Excluir a un cliente que sí se puede atender.
    r = good;
    r.trayectoria = {good.trayectoria.front()};
    r.orden = {"D0"};
    r.costo = 0;
    r.recargas = 0;
    r.excluidos = {{"C1", "BATTERY"}};
    expectRejected(hand, solver, r, "C1 se excluyo por BATTERY pero se puede atender");

    std::cout << "  -> PASSED\n" << std::endl;
}

void testVerdicts() {
    std::cout << "[TEST] 3. Veredictos OK, WA, TLE y MLE..." << std::endl;

    LimitesCategoria limits{1.0, 256.0};
    assert(Judge::verdictFor(true, 999.0, 100.0, limits) == "OK");
    assert(Judge::verdictFor(false, 1.0, 1.0, limits) == "WA");
    assert(Judge::verdictFor(false, 5000.0, 900.0, limits) == "WA");   // Una ruta inválida es WA aunque tarde
    assert(Judge::verdictFor(true, 1000.1, 100.0, limits) == "TLE");
    assert(Judge::verdictFor(true, 999.0, 256.5, limits) == "MLE");

    std::cout << "  -> PASSED\n" << std::endl;
}

JudgeRow makeRow(Categoria categoria, const std::string& algoritmo, const std::string& verdict,
                 double gap, double timeMs) {
    JudgeRow row;
    row.categoria = categoria;
    row.algoritmo = algoritmo;
    row.verdict = verdict;
    row.gapBest = gap;
    row.timeMs = timeMs;
    return row;
}

void testRanking() {
    std::cout << "[TEST] 4. Ranking: mas casos OK, luego menor gap, luego menor tiempo..." << std::endl;

    std::vector<JudgeRow> rows = {
        makeRow(Categoria::SMALL, "lento", "OK", 0.0, 10.0),
        makeRow(Categoria::SMALL, "lento", "OK", 2.0, 10.0),
        makeRow(Categoria::SMALL, "fallado", "OK", 0.0, 1.0),
        makeRow(Categoria::SMALL, "fallado", "WA", 0.0, 1.0),
        makeRow(Categoria::SMALL, "rapido", "OK", 0.0, 5.0),
        makeRow(Categoria::SMALL, "rapido", "OK", 2.0, 5.0),
        makeRow(Categoria::SMALL, "preciso", "OK", 0.0, 50.0),
        makeRow(Categoria::SMALL, "preciso", "OK", 0.0, 50.0),
        makeRow(Categoria::MEDIUM, "rapido", "OK", 3.0, 7.0),
    };
    std::vector<RankingRow> ranking = Judge::rank(rows);
    assert(ranking.size() == 5);

    // Small: preciso (2 OK, gap 0) > rapido (2 OK, gap 1, 5 ms) > lento (2 OK, gap 1, 10 ms) > fallado (1 OK).
    const std::vector<std::string> expected = {"preciso", "rapido", "lento", "fallado"};
    for (size_t i = 0; i < expected.size(); ++i) {
        assert(ranking[i].categoria == Categoria::SMALL);
        assert(ranking[i].algoritmo == expected[i] && ranking[i].position == static_cast<int>(i) + 1);
    }
    assert(ranking[1].okCases == 2 && ranking[1].cases == 2 && std::abs(ranking[1].meanGap - 1.0) < 1e-9);
    assert(ranking[3].okCases == 1 && ranking[3].cases == 2);
    assert(ranking[4].categoria == Categoria::MEDIUM && ranking[4].position == 1);

    std::cout << "  -> PASSED\n" << std::endl;
}

void testRunEndToEnd(const std::string& folder) {
    std::cout << "[TEST] 5. Benchmark completo sobre los casos adversariales..." << std::endl;

    std::vector<std::string> files = Judge::findScenarioFiles(folder);
    assert(files.size() == 5);                                    // Solo los .txt, no el README
    assert(Judge::findScenarioFiles("carpeta_que_no_existe").empty());

    const std::string output = "tmp_test_judge";
    std::filesystem::remove_all(output);
    std::ostringstream log;
    std::vector<JudgeRow> rows = Judge::run(files, output + "/routes", log);

    // 5 casos x 6 algoritmos (en los adversariales el exacto sí corre).
    assert(rows.size() == 30);
    std::vector<std::string> routeFiles = Judge::findScenarioFiles(output + "/routes");
    assert(routeFiles.size() == 30);
    for (const JudgeRow& row : rows) {
        assert(row.verdict == "OK");
        assert(row.gapBest >= -1e-9 && row.gapBound >= -1e-9);
    }
    // Cada caso tiene al menos una ruta con gap 0 contra la mejor.
    for (const std::string& file : files) {
        std::string nombre = ArchivoEscenario::cargar(file).nombre;
        bool hasBest = std::any_of(rows.begin(), rows.end(), [&](const JudgeRow& row) {
            return row.escenario == nombre && std::abs(row.gapBest) < 1e-9;
        });
        assert(hasBest);
    }

    std::ostringstream csv;
    Exporter::writeResultsCsv(rows, csv);
    std::string text = csv.str();
    assert(text.rfind("escenario,categoria,clientes,algoritmo,veredicto,costo,recargas,excluidos,tiempo_ms,"
                      "memoria_mb,gap_mejor,gap_cota\n", 0) == 0);
    assert(std::count(text.begin(), text.end(), '\n') == 31);

    std::vector<RankingRow> ranking = Judge::rank(rows);
    assert(ranking.size() == 6 && ranking.front().position == 1 && ranking.back().position == 6);
    for (const RankingRow& row : ranking) assert(row.okCases == 5 && row.cases == 5);

    std::filesystem::remove_all(output);

    std::cout << "  -> PASSED\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string adversarialFolder = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DEL JUEZ                 " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    testAcceptsValidRoutes(adversarialFolder);
    testRejectsInvalidRoutes(adversarialFolder);
    testVerdicts();
    testRanking();
    testRunEndToEnd(adversarialFolder);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DEL JUEZ PASARON CON EXITO          " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
