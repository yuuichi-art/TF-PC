// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/solvers/SolverExacto.hpp"
#include "../src/solvers/SolverMision.hpp"
#include "../src/exportacion/Exportador.hpp"
#include "../src/escenario/GeneradorDatasets.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"
#include "../src/geometria/Geometria.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;

const double TOLERANCIA = 1e-6;

bool esEtiquetaRecarga(const std::string& etiqueta) { return etiqueta[0] == 'D' || etiqueta[0] == 'S'; }

// Verificación independiente del solver, solo con la ruta y el escenario: lo mismo que hará el juez.
void verificarRutaValida(const Escenario& s, const Ruta& ruta, size_t clientesEsperados) {
    assert(ruta.factible);
    assert(!ruta.trayectoria.empty() && ruta.trayectoria.front().etiqueta == "D0" &&
           ruta.trayectoria.back().etiqueta == "D0");
    assert(ruta.orden.front() == "D0" && ruta.orden.back() == "D0");
    assert(ruta.preparacionMs >= 0 && ruta.resolucionMs >= 0);

    double bateria = s.bateria;
    double longitud = 0.0;
    int recargas = 0;
    std::set<std::string> visitados;
    std::vector<std::string> etiquetasClave = {ruta.trayectoria.front().etiqueta};
    for (size_t i = 1; i < ruta.trayectoria.size(); ++i) {
        Segmento tramo(ruta.trayectoria[i - 1].punto, ruta.trayectoria[i].punto);
        assert(Geometria::tramoLibre(tramo, s.obstaculos));   // Ningún tramo cruza un obstáculo
        longitud += tramo.longitud();
        bateria -= tramo.longitud();
        assert(bateria >= -TOLERANCIA);                                 // La batería nunca baja de 0

        const std::string& etiqueta = ruta.trayectoria[i].etiqueta;
        if (etiqueta == "V") continue;
        etiquetasClave.push_back(etiqueta);
        if (esEtiquetaRecarga(etiqueta)) {
            bateria = s.bateria;
            if (i + 1 < ruta.trayectoria.size()) ++recargas;
        } else {
            assert(visitados.insert(etiqueta).second);                      // Cada cliente una sola vez
        }
    }
    assert(std::abs(longitud - ruta.costo) < TOLERANCIA);
    assert(recargas == ruta.recargas);
    assert(visitados.size() == clientesEsperados);
    assert(etiquetasClave == ruta.orden);                                  // La trayectoria sigue el orden
}

Escenario escenarioAMano() {
    // Base (0,0), estación (10,0), cliente (10,6), batería 12: la ruta óptima es
    // D0 -> S1 -> C1 -> S1 -> D0 = 32, con dos recargas (calculada a mano en prueba_red).
    Escenario s;
    s.nombre = "hand";
    s.categoria = Categoria::ADVERSARIAL;
    s.bateria = 12;
    s.base = Punto(0, 0);
    s.estaciones = {Punto(10, 0)};
    s.clientes = {Punto(10, 6)};
    return s;
}

void probarRutaAManoYExportacion() {
    std::cout << "[PRUEBA] 1. Ruta calculada a mano y su archivo exportado..." << std::endl;

    Escenario s = escenarioAMano();
    SolverMision solver(s);
    Ruta ruta = solver.resolver(Algoritmo::BRANCH_AND_BOUND);
    verificarRutaValida(s, ruta, 1);
    assert(ruta.orden == std::vector<std::string>({"D0", "S1", "C1", "S1", "D0"}));
    assert(std::abs(ruta.costo - 32.0) < TOLERANCIA && ruta.recargas == 2);

    std::ostringstream salida;
    Exportador::escribirRuta(ruta, salida);
    const std::string esperado =
        "SCENARIO hand\n"
        "ALGORITHM branch_and_bound\n"
        "STATUS OK\n"
        "COST 32.000000\n"
        "RECHARGES 2\n"
        "EXCLUDED 0\n"
        "ORDER 5\n"
        "D0 S1 C1 S1 D0\n"
        "PATH 5\n"
        "0 0 D0\n"
        "10 0 S1\n"
        "10 6 C1\n"
        "10 0 S1\n"
        "0 0 D0\n";
    assert(salida.str() == esperado);

    // guardarRuta crea la carpeta y escribe lo mismo.
    const std::string carpeta = "tmp_prueba_mission";
    std::filesystem::remove_all(carpeta);
    Exportador::guardarRuta(ruta, carpeta + "/ruta.txt");
    std::ifstream archivo(carpeta + "/ruta.txt");
    std::stringstream contenido;
    contenido << archivo.rdbuf();
    archivo.close();
    assert(contenido.str() == esperado);
    std::filesystem::remove_all(carpeta);

    std::cout << "  -> OK\n" << std::endl;
}

void probarTodosDanRutasValidas(const std::string& carpeta) {
    std::cout << "[PRUEBA] 2. Todos los algoritmos dan rutas validas (adversariales y generados)..." << std::endl;

    std::vector<Escenario> escenarios;
    for (const char* nombre : {"adv_collinear", "adv_enclosed", "adv_corridor", "adv_concave_trap",
                               "adv_unreachable_battery"}) {
        escenarios.push_back(ArchivoEscenario::cargar(carpeta + "/" + nombre + ".txt"));
    }
    for (uint64_t semilla = 1; semilla <= 2; ++semilla) {
        escenarios.push_back(GeneradorDatasets::generar(Categoria::SMALL, semilla));
        escenarios.push_back(GeneradorDatasets::generar(Categoria::MEDIUM, semilla));
    }

    for (const Escenario& s : escenarios) {
        SolverMision solver(s);
        assert(solver.clientesAtendibles().size() + solver.excluidos().size() == static_cast<size_t>(s.numClientes()));
        for (Algoritmo algoritmo : todosLosAlgoritmos()) {
            if (algoritmo == Algoritmo::BRANCH_AND_BOUND && !solver.admiteExacto()) continue;
            Ruta ruta = solver.resolver(algoritmo);
            assert(ruta.algoritmo == nombreAlgoritmo(algoritmo) && ruta.escenario == s.nombre);
            verificarRutaValida(s, ruta, solver.clientesAtendibles().size());
        }
    }

    std::cout << "  -> OK\n" << std::endl;
}

void probarExclusiones(const std::string& carpeta) {
    std::cout << "[PRUEBA] 3. Clientes excluidos por obstaculos y por bateria..." << std::endl;

    Escenario encerrado = ArchivoEscenario::cargar(carpeta + "/adv_enclosed.txt");
    SolverMision solverEncerrado(encerrado);
    assert(solverEncerrado.excluidos().size() == 1);
    assert(solverEncerrado.excluidos()[0].etiqueta == "C1" && solverEncerrado.excluidos()[0].motivo == "OBSTACLES");
    verificarRutaValida(encerrado, solverEncerrado.resolver(Algoritmo::BRANCH_AND_BOUND), 2);

    Escenario bateria = ArchivoEscenario::cargar(carpeta + "/adv_unreachable_battery.txt");
    SolverMision solverBateria(bateria);
    assert(solverBateria.excluidos().size() == 1);
    assert(solverBateria.excluidos()[0].etiqueta == "C4" && solverBateria.excluidos()[0].motivo == "BATTERY");
    Ruta ruta = solverBateria.resolver(Algoritmo::BUSQUEDA_LOCAL_ITERADA);
    verificarRutaValida(bateria, ruta, 3);
    assert(ruta.excluidos.size() == 1);

    std::cout << "  -> OK\n" << std::endl;
}

void probarDisponibilidadExacto() {
    std::cout << "[PRUEBA] 4. El exacto solo se ofrece con pocos clientes..." << std::endl;

    Escenario mediano = GeneradorDatasets::generar(Categoria::MEDIUM, 1);
    assert(mediano.numClientes() > SolverExacto::MAX_CLIENTES);
    SolverMision solver(mediano);
    assert(!solver.admiteExacto());
    bool rechazado = false;
    try {
        solver.resolver(Algoritmo::BRANCH_AND_BOUND);
    } catch (const std::invalid_argument&) {
        rechazado = true;
    }
    assert(rechazado);

    std::cout << "  -> OK\n" << std::endl;
}

void probarSinClientes() {
    std::cout << "[PRUEBA] 5. Sin clientes el dron no despega..." << std::endl;

    Escenario s = escenarioAMano();
    s.clientes.clear();
    SolverMision solver(s);
    for (Algoritmo algoritmo : todosLosAlgoritmos()) {
        Ruta ruta = solver.resolver(algoritmo);
        assert(ruta.factible && ruta.costo == 0.0 && ruta.recargas == 0);
        assert(ruta.orden == std::vector<std::string>{"D0"} && ruta.trayectoria.size() == 1);
    }

    std::cout << "  -> OK\n" << std::endl;
}

size_t contarApariciones(const std::string& texto, const std::string& fragmento) {
    size_t cantidad = 0;
    for (size_t pos = texto.find(fragmento); pos != std::string::npos; pos = texto.find(fragmento, pos + 1)) ++cantidad;
    return cantidad;
}

std::string svgDe(const Escenario& s, const Ruta& ruta) {
    std::ostringstream salida;
    Exportador::escribirSvg(s, ruta, salida);
    return salida.str();
}

void probarSvg(const std::string& carpeta) {
    std::cout << "[PRUEBA] 6. Dibujo SVG del escenario y la ruta..." << std::endl;

    // Ruta a mano: un SVG completo con un elemento de cada tipo y la trayectoria de 5 puntos.
    Escenario aMano = escenarioAMano();
    SolverMision solverAMano(aMano);
    std::string svg = svgDe(aMano, solverAMano.resolver(Algoritmo::BRANCH_AND_BOUND));
    assert(svg.rfind("<svg xmlns=\"http://www.w3.org/2000/svg\"", 0) == 0);
    assert(svg.size() >= 7 && svg.compare(svg.size() - 7, 7, "</svg>\n") == 0);
    assert(contarApariciones(svg, "class=\"route\"") == 1 && contarApariciones(svg, "class=\"depot\"") == 1);
    assert(contarApariciones(svg, "class=\"station\"") == 1 && contarApariciones(svg, "class=\"client\"") == 1);
    assert(contarApariciones(svg, "class=\"obstacle\"") == 0 && contarApariciones(svg, "class=\"excluded\"") == 0);
    size_t inicio = svg.find("points=\"", svg.find("class=\"route\"")) + 8;
    std::string puntos = svg.substr(inicio, svg.find('"', inicio) - inicio);
    assert(std::count(puntos.begin(), puntos.end(), ' ') + 1 == 5);
    // Área de 1000 x 1000 en 800 px (escala 0,8), margen 20 y encabezado 40: la base (0,0) queda
    // en el píxel (20, 840) y su cuadrado de 12 px empieza en (14, 834). El eje y se invierte.
    assert(svg.find("class=\"depot\" x=\"14.00\" y=\"834.00\"") != std::string::npos);
    assert(svg.find(">C1</text>") != std::string::npos && svg.find("costo 32.00") != std::string::npos);

    // Cliente encerrado: 4 obstáculos, el excluido como cruz y los otros dos como círculos.
    Escenario encerrado = ArchivoEscenario::cargar(carpeta + "/adv_enclosed.txt");
    SolverMision solverEncerrado(encerrado);
    svg = svgDe(encerrado, solverEncerrado.resolver(Algoritmo::BRANCH_AND_BOUND));
    assert(contarApariciones(svg, "class=\"obstacle\"") == 4);
    assert(contarApariciones(svg, "class=\"excluded\"") == 1 && contarApariciones(svg, "class=\"client\"") == 2);

    // Con más de 40 clientes se omiten las etiquetas para que el dibujo se lea.
    Escenario grande = GeneradorDatasets::generar(Categoria::LARGE, 1);
    SolverMision solverGrande(grande);
    svg = svgDe(grande, solverGrande.resolver(Algoritmo::VECINO_MAS_CERCANO));
    assert(contarApariciones(svg, "class=\"client\"") == static_cast<size_t>(grande.numClientes()));
    assert(svg.find(">C1</text>") == std::string::npos);

    // guardarSvg crea la carpeta y el archivo.
    const std::string carpetaSalida = "tmp_prueba_svg";
    std::filesystem::remove_all(carpetaSalida);
    Exportador::guardarSvg(aMano, solverAMano.resolver(Algoritmo::VECINO_MAS_CERCANO), carpetaSalida + "/ruta.svg");
    assert(std::filesystem::file_size(carpetaSalida + "/ruta.svg") > 500);
    std::filesystem::remove_all(carpetaSalida);

    std::cout << "  -> OK\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string carpetaAdversarial = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DE LA MISION COMPLETA    " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    probarRutaAManoYExportacion();
    probarTodosDanRutasValidas(carpetaAdversarial);
    probarExclusiones(carpetaAdversarial);
    probarDisponibilidadExacto();
    probarSinClientes();
    probarSvg(carpetaAdversarial);

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DE LA MISION PASARON CON EXITO      " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
