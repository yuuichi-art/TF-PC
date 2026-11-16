#include "grafo/RedRecarga.hpp"
#include "grafo/GrafoMision.hpp"
#include "juez/Juez.hpp"
#include "juez/UsoMemoria.hpp"
#include "exportacion/Exportador.hpp"
#include "escenario/GeneradorDatasets.hpp"
#include "escenario/ArchivoEscenario.hpp"
#include "solvers/SolverMision.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;

// ---------------------------------------------------------------------------------------------
// Entrada por teclado (decisión del menú, docs/decisiones.md §5)
// ---------------------------------------------------------------------------------------------

// Si la entrada se cierra (por ejemplo, al pasar las respuestas por tubería), se usan los
// valores por defecto y el menú termina en lugar de quedar en un bucle.
bool inputClosed = false;

// La memoria pico del proceso nunca baja: si antes del juez se hizo otro trabajo en esta
// sesión, la memoria de cada caso queda tapada por ese pico anterior.
bool sessionUsedMemory = false;

std::string trim(const std::string& text) {
    size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

// Lee una línea completa (las rutas pueden tener espacios). Enter acepta el valor por defecto.
// Con cin.tie(NULL) la pregunta no se mostraría antes de leer: por eso termina en flush.
std::string askText(const std::string& question, const std::string& defaultValue) {
    std::cout << question << " [" << defaultValue << "]: " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) {
        inputClosed = true;
        std::cout << '\n';
        return defaultValue;
    }
    line = trim(line);
    return line.empty() ? defaultValue : line;
}

// Pide un entero en [min, max] y repite la pregunta hasta recibir uno válido.
int askNumber(const std::string& question, int defaultValue, int min, int max) {
    while (true) {
        std::string text = askText(question, std::to_string(defaultValue));
        try {
            size_t used = 0;
            int value = std::stoi(text, &used);
            if (used == text.size() && value >= min && value <= max) return value;
        } catch (const std::exception&) {
            // Texto que no es un número: se vuelve a preguntar
        }
        if (inputClosed) return defaultValue;
        std::cout << "  Valor invalido: escriba un numero entre " << min << " y " << max << ".\n";
    }
}

// ---------------------------------------------------------------------------------------------
// Presentación
// ---------------------------------------------------------------------------------------------

void printBanner() {
    std::cout << "========================================================================\n";
    std::cout << "       UNIVERSIDAD SAN IGNACIO DE LOYOLA - FACULTAD DE INGENIERIA       \n";
    std::cout << "             PROGRAMACION COMPETITIVA (2026) - GRUPO 4                 \n";
    std::cout << "                       GEODRONE CHALLENGE                               \n";
    std::cout << "   Optimizacion de Rutas de Entrega con Drones en Entornos Urbanos      \n";
    std::cout << "========================================================================\n";
    std::cout << " Integrantes:\n";
    std::cout << "   - Caso Egaovil, Pedro Yuichi        (2411176)\n";
    std::cout << "   - Fernandez Totocayo, Henry Daniel  (2412247)\n";
    std::cout << "   - Patino Quispe, Renato Enrique     (2410049)\n";
    std::cout << "   - Chanca Melgar, Yarel Joshua       (2411997)\n";
    std::cout << " Docente: Jomark Pablo Noriega Zapata\n";
    std::cout << "========================================================================\n";
}

void printScenarioSummary(const Escenario& s) {
    std::cout << "  Escenario " << s.nombre << " (" << nombreCategoria(s.categoria) << ", semilla " << s.semilla << "): "
              << "clientes " << s.numClientes() << ", estaciones " << s.numEstaciones() << ", obstaculos "
              << s.numObstaculos() << " (" << s.totalVerticesObstaculos() << " vertices), bateria " << s.bateria << "\n";
}

// Lista de etiquetas; si es muy larga muestra el inicio y el final.
void printLabels(const std::vector<std::string>& labels) {
    const size_t head = 15, tail = 5;
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels.size() > head + tail && i == head) {
            std::cout << " ... (" << labels.size() - head - tail << " mas) ...";
            i = labels.size() - tail;
        }
        std::cout << ' ' << labels[i];
    }
    std::cout << '\n';
}

// ---------------------------------------------------------------------------------------------
// Opción 1: generar datasets
// ---------------------------------------------------------------------------------------------

void generateDatasets() {
    std::cout << "\n--- GENERAR DATASETS ---\n";
    std::cout << "  Categoria  Clientes   Estaciones  Obstaculos  Tiempo  Memoria\n";
    for (Categoria c : {Categoria::SMALL, Categoria::MEDIUM, Categoria::LARGE,
                              Categoria::EXTREME}) {
        RangosCategoria spec = GeneradorDatasets::rangosDe(c);
        LimitesCategoria limits = limitesDe(c);
        std::cout << "  " << std::left << std::setw(10) << nombreCategoria(c) << std::right
                  << std::setw(5) << spec.minClientes << "-" << std::left << std::setw(6) << spec.maxClientes
                  << std::right << std::setw(5) << spec.minEstaciones << "-" << std::left << std::setw(6) << spec.maxEstaciones
                  << std::right << std::setw(5) << spec.minObstaculos << "-" << std::left << std::setw(6) << spec.maxObstaculos
                  << std::right << std::setw(4) << limits.limiteTiempoSegundos << " s" << std::setw(6)
                  << limits.limiteMemoriaMB << " MB\n";
    }

    std::string folder = askText("Carpeta de destino", "data/generated");
    int seeds = askNumber("Semillas por categoria", 5, 1, 50);
    std::cout << "  Generando (cada escenario calcula su bateria con la busqueda binaria)...\n" << std::flush;

    std::vector<std::string> paths = GeneradorDatasets::generarConjuntoBenchmark(folder, seeds);
    for (const std::string& path : paths) {
        printScenarioSummary(ArchivoEscenario::cargar(path));
    }
    std::cout << "  " << paths.size() << " escenarios guardados en " << folder << "\n";
}

// ---------------------------------------------------------------------------------------------
// Opción 2: resolver un escenario
// ---------------------------------------------------------------------------------------------

void solveScenario() {
    std::cout << "\n--- RESOLVER UN ESCENARIO ---\n";
    std::string path = askText("Archivo de escenario", "data/generated/small_1.txt");
    Escenario escenario = ArchivoEscenario::cargar(path);
    printScenarioSummary(escenario);

    SolverMision solver(escenario);
    std::cout << std::fixed << std::setprecision(2)
              << "  Grafo de mision y red de recarga listos en " << solver.preparacionMs() << " ms\n";

    // Opciones 1..count; el exacto solo está disponible con pocos clientes atendibles.
    const std::vector<Algoritmo>& algorithms = todosLosAlgoritmos();
    int count = static_cast<int>(algorithms.size());
    auto available = [&](int option) {
        return algorithms[option - 1] != Algoritmo::BRANCH_AND_BOUND || solver.admiteExacto();
    };
    for (int option = 1; option <= count; ++option) {
        std::cout << "  " << option << ". " << nombreAlgoritmo(algorithms[option - 1])
                  << (available(option) ? "" : "  (no disponible: demasiados clientes para el exacto)") << '\n';
    }
    // El valor por defecto siempre está disponible (el exacto o la búsqueda local iterada), así que el
    // bucle termina aunque se cierre la entrada: askNumber devuelve entonces ese valor.
    int defaultChoice = solver.admiteExacto() ? 1 : count;
    int choice = askNumber("Algoritmo", defaultChoice, 1, count);
    while (!available(choice)) {
        std::cout << "  El exacto no esta disponible para este escenario.\n";
        choice = askNumber("Algoritmo", defaultChoice, 1, count);
    }

    Ruta ruta = solver.resolver(algorithms[choice - 1]);
    LimitesCategoria limits = limitesDe(escenario.categoria);

    std::cout << "\n  Algoritmo:  " << ruta.algoritmo << "\n";
    std::cout << "  Estado:     " << (ruta.factible ? "OK" : "INFACTIBLE") << "\n";
    std::cout << std::setprecision(2) << "  Costo:      " << ruta.costo << " (distancia volada, con recargas)\n";
    std::cout << "  Recargas:   " << ruta.recargas << "\n";
    std::cout << "  Excluidos:  " << ruta.excluidos.size();
    for (const ClienteExcluido& client : ruta.excluidos) std::cout << "  " << client.etiqueta << " (" << client.motivo << ")";
    std::cout << '\n';
    std::cout << "  Tiempo:     " << ruta.totalMs() << " ms = " << ruta.preparacionMs << " de preparacion + "
              << ruta.resolucionMs << " de resolucion (limite " << static_cast<int>(limits.limiteTiempoSegundos * 1000)
              << " ms)\n";
    std::cout << "  Memoria:    " << peakMemoryMB() << " MB pico del proceso (limite "
              << static_cast<int>(limits.limiteMemoriaMB) << " MB)\n";
    std::cout << "  Orden:     ";
    printLabels(ruta.orden);

    std::string output = "output/" + ruta.escenario + "_" + ruta.algoritmo;
    Exporter::saveRoute(ruta, output + ".txt");
    Exporter::saveSvg(escenario, ruta, output + ".svg");
    std::cout << "  Ruta completa (" << ruta.trayectoria.size() << " puntos) exportada en " << output << ".txt\n"
              << "  Dibujo de la ruta en " << output << ".svg (se abre con cualquier navegador)\n";
}

// ---------------------------------------------------------------------------------------------
// Opción 3: juez y benchmark (RF14, RF15)
// ---------------------------------------------------------------------------------------------

void printRanking(const std::vector<RankingRow>& ranking) {
    int categoria = -1;
    for (const RankingRow& row : ranking) {
        if (static_cast<int>(row.categoria) != categoria) {
            categoria = static_cast<int>(row.categoria);
            std::cout << "\n  " << nombreCategoria(row.categoria) << "\n"
                      << "    #  algoritmo                 OK/casos   gap medio   tiempo medio\n";
        }
        std::cout << "    " << row.position << "  " << std::left << std::setw(26) << row.algoritmo << std::right
                  << std::setw(3) << row.okCases << "/" << std::left << std::setw(6) << row.cases << std::right
                  << std::fixed << std::setprecision(2) << std::setw(8) << row.meanGap << " %"
                  << std::setw(12) << row.meanTimeMs << " ms\n";
    }
}

void runJudge() {
    std::cout << "\n--- JUEZ Y BENCHMARK ---\n";
    std::string folder = askText("Carpeta de escenarios", "data/generated");
    std::string adversarial = askText("Incluir los casos de data/adversarial (s/n)", "s");

    std::vector<std::string> files = Judge::findScenarioFiles(folder);
    if (!adversarial.empty() && (adversarial[0] == 's' || adversarial[0] == 'S')) {
        std::vector<std::string> extra = Judge::findScenarioFiles("data/adversarial");
        files.insert(files.end(), extra.begin(), extra.end());
    }
    if (files.empty()) {
        std::cout << "  No hay escenarios en " << folder << ". Genere datasets con la opcion 1.\n";
        return;
    }

    const std::string output = "output/judge";
    if (sessionUsedMemory) {
        std::cout << "  Aviso: en esta sesion ya se uso memoria (pico actual " << std::fixed << std::setprecision(1)
                  << peakMemoryMB() << " MB). La columna de memoria no sera por caso: para medirla,\n"
                  << "  ejecute el juez como primera opcion al abrir el programa.\n";
    }
    std::cout << "  Ejecutando " << files.size() << " escenarios de menor a mayor...\n" << std::flush;
    std::vector<JudgeRow> rows = Judge::run(files, output + "/routes", std::cout);
    std::vector<RankingRow> ranking = Judge::rank(rows);
    Exporter::saveResultsCsv(rows, output + "/results.csv");
    Exporter::saveRankingCsv(ranking, output + "/ranking.csv");

    int ok = 0, wa = 0, tle = 0, mle = 0;
    for (const JudgeRow& row : rows) {
        ok += row.verdict == "OK";
        wa += row.verdict == "WA";
        tle += row.verdict == "TLE";
        mle += row.verdict == "MLE";
    }
    std::cout << "\n  Veredictos: " << ok << " OK, " << wa << " WA, " << tle << " TLE, " << mle << " MLE (de "
              << rows.size() << " ejecuciones)\n";
    std::cout << "  Ranking por categoria (mas casos OK, luego menor gap medio, luego menor tiempo):";
    printRanking(ranking);
    std::cout << "\n  Resultados en " << output << "/results.csv, ranking en " << output
              << "/ranking.csv y rutas en " << output << "/routes/\n";
}

// ---------------------------------------------------------------------------------------------
// Opción 4: analizar la red de recarga (RF10, RF11, RF12)
// ---------------------------------------------------------------------------------------------

void analyzeNetwork() {
    std::cout << "\n--- ANALIZAR RED DE RECARGA ---\n";
    std::string path = askText("Archivo de escenario", "data/generated/medium_1.txt");
    Escenario escenario = ArchivoEscenario::cargar(path);
    printScenarioSummary(escenario);

    // El solver prepara el grafo de misión y la red de recarga, y clasifica a los excluidos.
    SolverMision solver(escenario);
    const GrafoMision& mision = solver.mision();
    const RedRecarga& red = solver.red();
    std::cout << std::fixed << std::setprecision(2);

    // Alcance de la red con la batería del escenario (DFS desde la base).
    std::vector<std::string> unreachable;
    for (int s = 0; s < mision.numEstaciones(); ++s) {
        int key = mision.claveEstacion(s);
        if (!red.esAlcanzable(key)) unreachable.push_back(mision.etiqueta(key));
    }
    std::cout << "  Estaciones alcanzables desde la base: " << mision.numEstaciones() - unreachable.size()
              << " de " << mision.numEstaciones() << '\n';
    if (!unreachable.empty()) {
        std::cout << "  Sin conexion con la base:";
        printLabels(unreachable);
    }

    // Estaciones críticas: puntos de articulación (Tarjan).
    std::vector<std::string> critical;
    for (int key : red.estacionesCriticas()) critical.push_back(mision.etiqueta(key));
    std::cout << "  Estaciones criticas (puntos de articulacion): ";
    if (critical.empty()) {
        std::cout << "ninguna\n";
    } else {
        printLabels(critical);
    }

    // Red mínima de soporte: árbol de expansión mínima entre la base y las estaciones (Prim).
    std::vector<int> nodes;
    ArbolMinimo tree = RedRecarga::arbolSoporte(mision, nodes);
    std::cout << "  Red minima de soporte: costo " << tree.costo << ", tramo mas largo " << tree.aristaMaxima << '\n';
    for (size_t i = 1; i < nodes.size(); ++i) {
        if (tree.padre[i] < 0) continue;
        int from = nodes[tree.padre[i]];
        std::cout << "    " << mision.etiqueta(from) << " - " << mision.etiqueta(nodes[i])
                  << "  " << mision.distancia(from, nodes[i]) << '\n';
    }

    // Autonomía mínima (búsqueda binaria) frente a la del escenario.
    double minimum = RedRecarga::autonomiaMinima(mision);
    std::cout << "  Autonomia minima para atender a todos los clientes: " << minimum
              << " (el escenario tiene " << escenario.bateria << ")\n";

    std::vector<std::string> byObstacles, byBattery;
    for (const ClienteExcluido& client : solver.excluidos()) {
        if (client.motivo == "OBSTACLES") {
            byObstacles.push_back(client.etiqueta);
        } else {
            byBattery.push_back(client.etiqueta);
        }
    }
    std::cout << "  Clientes separados por obstaculos: " << byObstacles.size();
    if (!byObstacles.empty()) printLabels(byObstacles); else std::cout << '\n';
    std::cout << "  Clientes fuera del alcance de la bateria: " << byBattery.size();
    if (!byBattery.empty()) printLabels(byBattery); else std::cout << '\n';
}

// ---------------------------------------------------------------------------------------------

int main() {
    // I/O rápido de programación competitiva; las preguntas del menú hacen flush explícito.
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    printBanner();
    while (true) {
        std::cout << "\n=== MENU ===\n"
                  << "  1. Generar datasets\n"
                  << "  2. Resolver un escenario\n"
                  << "  3. Ejecutar juez y benchmark\n"
                  << "  4. Analizar red de recarga\n"
                  << "  0. Salir\n";
        int option = askNumber("Opcion", 0, 0, 4);
        if (option == 0) break;
        // Cada opción empieza con el formato normal de números: las otras dejan std::fixed activo.
        std::cout << std::defaultfloat << std::setprecision(6);
        try {
            if (option == 1) generateDatasets();
            if (option == 2) solveScenario();
            if (option == 3) runJudge();
            if (option == 4) analyzeNetwork();
        } catch (const std::exception& error) {
            std::cout << "  [ERROR] " << error.what() << '\n';
        }
        sessionUsedMemory = true;
        if (inputClosed) break;
    }
    std::cout << "Hasta luego.\n";
    return 0;
}
