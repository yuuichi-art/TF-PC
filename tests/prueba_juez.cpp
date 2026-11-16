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

Escenario escenarioAMano() {
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
void esperarRechazo(const Escenario& s, const SolverMision& solver, const Ruta& ruta, const std::string& fragmento) {
    ResultadoVerificacion resultado = Juez::verificarRuta(s, solver, ruta);
    if (resultado.valida || resultado.problema.find(fragmento) == std::string::npos) {
        std::cerr << "  Resultado: " << (resultado.valida ? "valida" : resultado.problema) << "\n  Se esperaba: " << fragmento << "\n";
        assert(false);
    }
}

void quitarPunto(Ruta& ruta, size_t indice) {
    ruta.trayectoria.erase(ruta.trayectoria.begin() + static_cast<long>(indice));
}

void probarAceptaRutasValidas(const std::string& carpeta) {
    std::cout << "[TEST] 1. El verificador acepta las rutas de todos los algoritmos..." << std::endl;

    std::vector<Escenario> escenarios;
    for (const std::string& archivo : Juez::buscarEscenarios(carpeta)) escenarios.push_back(ArchivoEscenario::cargar(archivo));
    escenarios.push_back(GeneradorDatasets::generar(Categoria::SMALL, 1));
    escenarios.push_back(GeneradorDatasets::generar(Categoria::MEDIUM, 1));
    escenarios.push_back(escenarioAMano());

    for (const Escenario& s : escenarios) {
        SolverMision solver(s);
        for (Algoritmo algoritmo : todosLosAlgoritmos()) {
            if (algoritmo == Algoritmo::BRANCH_AND_BOUND && !solver.admiteExacto()) continue;
            ResultadoVerificacion resultado = Juez::verificarRuta(s, solver, solver.resolver(algoritmo));
            if (!resultado.valida) std::cerr << "  " << s.nombre << ": " << resultado.problema << "\n";
            assert(resultado.valida);
        }
    }

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarRechazaRutasInvalidas(const std::string& carpeta) {
    std::cout << "[TEST] 2. El verificador rechaza cada tipo de ruta invalida..." << std::endl;

    // Trampa en U: sin los vértices de obstáculo, la ruta atraviesa la U.
    Escenario trampa = ArchivoEscenario::cargar(carpeta + "/adv_concave_trap.txt");
    SolverMision solverTrampa(trampa);
    Ruta recta = solverTrampa.resolver(Algoritmo::BRANCH_AND_BOUND);
    recta.trayectoria.erase(std::remove_if(recta.trayectoria.begin(), recta.trayectoria.end(),
                                       [](const PuntoRuta& p) { return p.etiqueta == "V"; }),
                        recta.trayectoria.end());
    esperarRechazo(trampa, solverTrampa, recta, "cruza un obstaculo");

    // Ruta a mano D0 S1 C1 S1 D0, que es válida, y variantes rotas.
    Escenario aMano = escenarioAMano();
    SolverMision solver(aMano);
    const Ruta buena = solver.resolver(Algoritmo::BRANCH_AND_BOUND);
    assert(Juez::verificarRuta(aMano, solver, buena).valida);

    Escenario debil = aMano;
    debil.bateria = 8;                                   // Con 8 no alcanza el tramo de 10 a la estación
    esperarRechazo(debil, solver, buena, "bateria se agota");

    Ruta r = buena;
    quitarPunto(r, 2);                                  // Sin C1
    esperarRechazo(aMano, solver, r, "C1 no se visita ni esta excluido");

    r = buena;
    r.trayectoria.insert(r.trayectoria.begin() + 2, r.trayectoria[2]);       // C1 dos veces
    esperarRechazo(aMano, solver, r, "C1 se visita mas de una vez");

    r = buena;
    r.trayectoria[2].punto.x += 1;                             // C1 fuera de su lugar
    esperarRechazo(aMano, solver, r, "C1 no esta en las coordenadas");

    r = buena;
    quitarPunto(r, 0);                                  // No empieza en la base
    esperarRechazo(aMano, solver, r, "no empieza y termina en la base");

    r = buena;
    r.costo += 5;
    esperarRechazo(aMano, solver, r, "costo informado");

    r = buena;
    r.recargas = 0;
    esperarRechazo(aMano, solver, r, "recargas informadas");

    r = buena;
    std::swap(r.orden[1], r.orden[2]);                  // ORDER no sigue a la trayectoria
    esperarRechazo(aMano, solver, r, "orden informado");

    r = buena;
    r.trayectoria[1].etiqueta = "X9";
    esperarRechazo(aMano, solver, r, "etiqueta desconocida");

    r = buena;
    r.factible = false;
    esperarRechazo(aMano, solver, r, "no encontro una ruta factible");

    // Excluir a un cliente que sí se puede atender.
    r = buena;
    r.trayectoria = {buena.trayectoria.front()};
    r.orden = {"D0"};
    r.costo = 0;
    r.recargas = 0;
    r.excluidos = {{"C1", "BATTERY"}};
    esperarRechazo(aMano, solver, r, "C1 se excluyo por BATTERY pero se puede atender");

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarVeredictos() {
    std::cout << "[TEST] 3. Veredictos OK, WA, TLE y MLE..." << std::endl;

    LimitesCategoria limits{1.0, 256.0};
    assert(Juez::veredictoPara(true, 999.0, 100.0, limits) == "OK");
    assert(Juez::veredictoPara(false, 1.0, 1.0, limits) == "WA");
    assert(Juez::veredictoPara(false, 5000.0, 900.0, limits) == "WA");   // Una ruta inválida es WA aunque tarde
    assert(Juez::veredictoPara(true, 1000.1, 100.0, limits) == "TLE");
    assert(Juez::veredictoPara(true, 999.0, 256.5, limits) == "MLE");

    std::cout << "  -> PASSED\n" << std::endl;
}

FilaJuez crearFila(Categoria categoria, const std::string& algoritmo, const std::string& veredicto,
                 double gap, double tiempoMs) {
    FilaJuez fila;
    fila.categoria = categoria;
    fila.algoritmo = algoritmo;
    fila.veredicto = veredicto;
    fila.gapMejor = gap;
    fila.tiempoMs = tiempoMs;
    return fila;
}

void probarRanking() {
    std::cout << "[TEST] 4. Ranking: mas casos OK, luego menor gap, luego menor tiempo..." << std::endl;

    std::vector<FilaJuez> filas = {
        crearFila(Categoria::SMALL, "lento", "OK", 0.0, 10.0),
        crearFila(Categoria::SMALL, "lento", "OK", 2.0, 10.0),
        crearFila(Categoria::SMALL, "fallado", "OK", 0.0, 1.0),
        crearFila(Categoria::SMALL, "fallado", "WA", 0.0, 1.0),
        crearFila(Categoria::SMALL, "rapido", "OK", 0.0, 5.0),
        crearFila(Categoria::SMALL, "rapido", "OK", 2.0, 5.0),
        crearFila(Categoria::SMALL, "preciso", "OK", 0.0, 50.0),
        crearFila(Categoria::SMALL, "preciso", "OK", 0.0, 50.0),
        crearFila(Categoria::MEDIUM, "rapido", "OK", 3.0, 7.0),
    };
    std::vector<FilaRanking> ranking = Juez::armarRanking(filas);
    assert(ranking.size() == 5);

    // Small: preciso (2 OK, gap 0) > rapido (2 OK, gap 1, 5 ms) > lento (2 OK, gap 1, 10 ms) > fallado (1 OK).
    const std::vector<std::string> esperado = {"preciso", "rapido", "lento", "fallado"};
    for (size_t i = 0; i < esperado.size(); ++i) {
        assert(ranking[i].categoria == Categoria::SMALL);
        assert(ranking[i].algoritmo == esperado[i] && ranking[i].posicion == static_cast<int>(i) + 1);
    }
    assert(ranking[1].casosOk == 2 && ranking[1].casos == 2 && std::abs(ranking[1].gapMedio - 1.0) < 1e-9);
    assert(ranking[3].casosOk == 1 && ranking[3].casos == 2);
    assert(ranking[4].categoria == Categoria::MEDIUM && ranking[4].posicion == 1);

    std::cout << "  -> PASSED\n" << std::endl;
}

void probarEjecucionCompleta(const std::string& carpeta) {
    std::cout << "[TEST] 5. Benchmark completo sobre los casos adversariales..." << std::endl;

    std::vector<std::string> archivos = Juez::buscarEscenarios(carpeta);
    assert(archivos.size() == 5);                                    // Solo los .txt, no el README
    assert(Juez::buscarEscenarios("carpeta_que_no_existe").empty());

    const std::string salida = "tmp_prueba_judge";
    std::filesystem::remove_all(salida);
    std::ostringstream registro;
    std::vector<FilaJuez> filas = Juez::ejecutar(archivos, salida + "/routes", registro);

    // 5 casos x 6 algoritmos (en los adversariales el exacto sí corre).
    assert(filas.size() == 30);
    std::vector<std::string> archivosRutas = Juez::buscarEscenarios(salida + "/routes");
    assert(archivosRutas.size() == 30);
    for (const FilaJuez& fila : filas) {
        assert(fila.veredicto == "OK");
        assert(fila.gapMejor >= -1e-9 && fila.gapCota >= -1e-9);
    }
    // Cada caso tiene al menos una ruta con gap 0 contra la mejor.
    for (const std::string& archivo : archivos) {
        std::string nombre = ArchivoEscenario::cargar(archivo).nombre;
        bool tieneMejor = std::any_of(filas.begin(), filas.end(), [&](const FilaJuez& fila) {
            return fila.escenario == nombre && std::abs(fila.gapMejor) < 1e-9;
        });
        assert(tieneMejor);
    }

    std::ostringstream csv;
    Exportador::escribirResultadosCsv(filas, csv);
    std::string texto = csv.str();
    assert(texto.rfind("escenario,categoria,clientes,algoritmo,veredicto,costo,recargas,excluidos,tiempo_ms,"
                      "memoria_mb,gap_mejor,gap_cota\n", 0) == 0);
    assert(std::count(texto.begin(), texto.end(), '\n') == 31);

    std::vector<FilaRanking> ranking = Juez::armarRanking(filas);
    assert(ranking.size() == 6 && ranking.front().posicion == 1 && ranking.back().posicion == 6);
    for (const FilaRanking& fila : ranking) assert(fila.casosOk == 5 && fila.casos == 5);

    std::filesystem::remove_all(salida);

    std::cout << "  -> PASSED\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string carpetaAdversarial = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DEL JUEZ                 " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    probarAceptaRutasValidas(carpetaAdversarial);
    probarRechazaRutasInvalidas(carpetaAdversarial);
    probarVeredictos();
    probarRanking();
    probarEjecucionCompleta(carpetaAdversarial);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DEL JUEZ PASARON CON EXITO          " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
