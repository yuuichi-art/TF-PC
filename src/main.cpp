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
bool entradaCerrada = false;

// La memoria pico del proceso nunca baja: si antes del juez se hizo otro trabajo en esta
// sesión, la memoria de cada caso queda tapada por ese pico anterior.
bool memoriaUsadaEnSesion = false;

std::string recortar(const std::string& texto) {
    size_t primero = texto.find_first_not_of(" \t\r\n");
    if (primero == std::string::npos) return "";
    size_t ultimo = texto.find_last_not_of(" \t\r\n");
    return texto.substr(primero, ultimo - primero + 1);
}

// Lee una línea completa (las rutas pueden tener espacios). Enter acepta el valor por defecto.
// Con cin.tie(NULL) la pregunta no se mostraría antes de leer: por eso termina en flush.
std::string preguntarTexto(const std::string& pregunta, const std::string& valorPorDefecto) {
    std::cout << pregunta << " [" << valorPorDefecto << "]: " << std::flush;
    std::string linea;
    if (!std::getline(std::cin, linea)) {
        entradaCerrada = true;
        std::cout << '\n';
        return valorPorDefecto;
    }
    linea = recortar(linea);
    return linea.empty() ? valorPorDefecto : linea;
}

// Pide un entero en [min, max] y repite la pregunta hasta recibir uno válido.
int preguntarNumero(const std::string& pregunta, int valorPorDefecto, int minimo, int maximo) {
    while (true) {
        std::string texto = preguntarTexto(pregunta, std::to_string(valorPorDefecto));
        try {
            size_t usados = 0;
            int valor = std::stoi(texto, &usados);
            if (usados == texto.size() && valor >= minimo && valor <= maximo) return valor;
        } catch (const std::exception&) {
            // Texto que no es un número: se vuelve a preguntar
        }
        if (entradaCerrada) return valorPorDefecto;
        std::cout << "  Valor invalido: escriba un numero entre " << minimo << " y " << maximo << ".\n";
    }
}

// ---------------------------------------------------------------------------------------------
// Presentación
// ---------------------------------------------------------------------------------------------

void imprimirPortada() {
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

void imprimirResumenEscenario(const Escenario& s) {
    std::cout << "  Escenario " << s.nombre << " (" << nombreCategoria(s.categoria) << ", semilla " << s.semilla << "): "
              << "clientes " << s.numClientes() << ", estaciones " << s.numEstaciones() << ", obstaculos "
              << s.numObstaculos() << " (" << s.totalVerticesObstaculos() << " vertices), bateria " << s.bateria << "\n";
}

// Lista de etiquetas; si es muy larga muestra el inicio y el final.
void imprimirEtiquetas(const std::vector<std::string>& etiquetas) {
    const size_t primeros = 15, ultimos = 5;
    for (size_t i = 0; i < etiquetas.size(); ++i) {
        if (etiquetas.size() > primeros + ultimos && i == primeros) {
            std::cout << " ... (" << etiquetas.size() - primeros - ultimos << " mas) ...";
            i = etiquetas.size() - ultimos;
        }
        std::cout << ' ' << etiquetas[i];
    }
    std::cout << '\n';
}

// ---------------------------------------------------------------------------------------------
// Opción 1: generar datasets
// ---------------------------------------------------------------------------------------------

void generarDatasets() {
    std::cout << "\n--- GENERAR DATASETS ---\n";
    std::cout << "  Categoria  Clientes   Estaciones  Obstaculos  Tiempo  Memoria\n";
    for (Categoria c : {Categoria::SMALL, Categoria::MEDIUM, Categoria::LARGE,
                              Categoria::EXTREME}) {
        RangosCategoria rangos = GeneradorDatasets::rangosDe(c);
        LimitesCategoria limites = limitesDe(c);
        std::cout << "  " << std::left << std::setw(10) << nombreCategoria(c) << std::right
                  << std::setw(5) << rangos.minClientes << "-" << std::left << std::setw(6) << rangos.maxClientes
                  << std::right << std::setw(5) << rangos.minEstaciones << "-" << std::left << std::setw(6) << rangos.maxEstaciones
                  << std::right << std::setw(5) << rangos.minObstaculos << "-" << std::left << std::setw(6) << rangos.maxObstaculos
                  << std::right << std::setw(4) << limites.limiteTiempoSegundos << " s" << std::setw(6)
                  << limites.limiteMemoriaMB << " MB\n";
    }

    std::string carpeta = preguntarTexto("Carpeta de destino", "data/generated");
    int semillas = preguntarNumero("Semillas por categoria", 5, 1, 50);
    std::cout << "  Generando (cada escenario calcula su bateria con la busqueda binaria)...\n" << std::flush;

    std::vector<std::string> archivos = GeneradorDatasets::generarConjuntoBenchmark(carpeta, semillas);
    for (const std::string& archivo : archivos) {
        imprimirResumenEscenario(ArchivoEscenario::cargar(archivo));
    }
    std::cout << "  " << archivos.size() << " escenarios guardados en " << carpeta << "\n";
}

// ---------------------------------------------------------------------------------------------
// Opción 2: resolver un escenario
// ---------------------------------------------------------------------------------------------

void resolverEscenario() {
    std::cout << "\n--- RESOLVER UN ESCENARIO ---\n";
    std::string archivo = preguntarTexto("Archivo de escenario", "data/generated/small_1.txt");
    Escenario escenario = ArchivoEscenario::cargar(archivo);
    imprimirResumenEscenario(escenario);

    SolverMision solver(escenario);
    std::cout << std::fixed << std::setprecision(2)
              << "  Grafo de mision y red de recarga listos en " << solver.preparacionMs() << " ms\n";

    // Opciones 1..count; el exacto solo está disponible con pocos clientes atendibles.
    const std::vector<Algoritmo>& algoritmos = todosLosAlgoritmos();
    int cantidad = static_cast<int>(algoritmos.size());
    auto disponible = [&](int opcion) {
        return algoritmos[opcion - 1] != Algoritmo::BRANCH_AND_BOUND || solver.admiteExacto();
    };
    for (int opcion = 1; opcion <= cantidad; ++opcion) {
        std::cout << "  " << opcion << ". " << nombreAlgoritmo(algoritmos[opcion - 1])
                  << (disponible(opcion) ? "" : "  (no disponible: demasiados clientes para el exacto)") << '\n';
    }
    // El valor por defecto siempre está disponible (el exacto o la búsqueda local iterada), así que el
    // bucle termina aunque se cierre la entrada: preguntarNumero devuelve entonces ese valor.
    int opcionPorDefecto = solver.admiteExacto() ? 1 : cantidad;
    int eleccion = preguntarNumero("Algoritmo", opcionPorDefecto, 1, cantidad);
    while (!disponible(eleccion)) {
        std::cout << "  El exacto no esta disponible para este escenario.\n";
        eleccion = preguntarNumero("Algoritmo", opcionPorDefecto, 1, cantidad);
    }

    Ruta ruta = solver.resolver(algoritmos[eleccion - 1]);
    LimitesCategoria limites = limitesDe(escenario.categoria);

    std::cout << "\n  Algoritmo:  " << ruta.algoritmo << "\n";
    std::cout << "  Estado:     " << (ruta.factible ? "OK" : "INFACTIBLE") << "\n";
    std::cout << std::setprecision(2) << "  Costo:      " << ruta.costo << " (distancia volada, con recargas)\n";
    std::cout << "  Recargas:   " << ruta.recargas << "\n";
    std::cout << "  Excluidos:  " << ruta.excluidos.size();
    for (const ClienteExcluido& cliente : ruta.excluidos) std::cout << "  " << cliente.etiqueta << " (" << cliente.motivo << ")";
    std::cout << '\n';
    std::cout << "  Tiempo:     " << ruta.totalMs() << " ms = " << ruta.preparacionMs << " de preparacion + "
              << ruta.resolucionMs << " de resolucion (limite " << static_cast<int>(limites.limiteTiempoSegundos * 1000)
              << " ms)\n";
    std::cout << "  Memoria:    " << memoriaPicoMB() << " MB pico del proceso (limite "
              << static_cast<int>(limites.limiteMemoriaMB) << " MB)\n";
    std::cout << "  Orden:     ";
    imprimirEtiquetas(ruta.orden);

    std::string salida = "output/" + ruta.escenario + "_" + ruta.algoritmo;
    Exportador::guardarRuta(ruta, salida + ".txt");
    Exportador::guardarSvg(escenario, ruta, salida + ".svg");
    std::cout << "  Ruta completa (" << ruta.trayectoria.size() << " puntos) exportada en " << salida << ".txt\n"
              << "  Dibujo de la ruta en " << salida << ".svg (se abre con cualquier navegador)\n";
}

// ---------------------------------------------------------------------------------------------
// Opción 3: juez y benchmark (RF14, RF15)
// ---------------------------------------------------------------------------------------------

void imprimirRanking(const std::vector<FilaRanking>& ranking) {
    int categoria = -1;
    for (const FilaRanking& fila : ranking) {
        if (static_cast<int>(fila.categoria) != categoria) {
            categoria = static_cast<int>(fila.categoria);
            std::cout << "\n  " << nombreCategoria(fila.categoria) << "\n"
                      << "    #  algoritmo                 OK/casos   gap medio   tiempo medio\n";
        }
        std::cout << "    " << fila.posicion << "  " << std::left << std::setw(26) << fila.algoritmo << std::right
                  << std::setw(3) << fila.casosOk << "/" << std::left << std::setw(6) << fila.casos << std::right
                  << std::fixed << std::setprecision(2) << std::setw(8) << fila.gapMedio << " %"
                  << std::setw(12) << fila.tiempoMedioMs << " ms\n";
    }
}

void ejecutarJuez() {
    std::cout << "\n--- JUEZ Y BENCHMARK ---\n";
    std::string carpeta = preguntarTexto("Carpeta de escenarios", "data/generated");
    std::string adversariales = preguntarTexto("Incluir los casos de data/adversarial (s/n)", "s");

    std::vector<std::string> archivos = Juez::buscarEscenarios(carpeta);
    if (!adversariales.empty() && (adversariales[0] == 's' || adversariales[0] == 'S')) {
        std::vector<std::string> extra = Juez::buscarEscenarios("data/adversarial");
        archivos.insert(archivos.end(), extra.begin(), extra.end());
    }
    if (archivos.empty()) {
        std::cout << "  No hay escenarios en " << carpeta << ". Genere datasets con la opcion 1.\n";
        return;
    }

    const std::string salida = "output/judge";
    if (memoriaUsadaEnSesion) {
        std::cout << "  Aviso: en esta sesion ya se uso memoria (pico actual " << std::fixed << std::setprecision(1)
                  << memoriaPicoMB() << " MB). La columna de memoria no sera por caso: para medirla,\n"
                  << "  ejecute el juez como primera opcion al abrir el programa.\n";
    }
    std::cout << "  Ejecutando " << archivos.size() << " escenarios de menor a mayor...\n" << std::flush;
    std::vector<FilaJuez> filas = Juez::ejecutar(archivos, salida + "/routes", std::cout);
    std::vector<FilaRanking> ranking = Juez::armarRanking(filas);
    Exportador::guardarResultadosCsv(filas, salida + "/results.csv");
    Exportador::guardarRankingCsv(ranking, salida + "/ranking.csv");

    int ok = 0, wa = 0, tle = 0, mle = 0;
    for (const FilaJuez& fila : filas) {
        ok += fila.veredicto == "OK";
        wa += fila.veredicto == "WA";
        tle += fila.veredicto == "TLE";
        mle += fila.veredicto == "MLE";
    }
    std::cout << "\n  Veredictos: " << ok << " OK, " << wa << " WA, " << tle << " TLE, " << mle << " MLE (de "
              << filas.size() << " ejecuciones)\n";
    std::cout << "  Ranking por categoria (mas casos OK, luego menor gap medio, luego menor tiempo):";
    imprimirRanking(ranking);
    std::cout << "\n  Resultados en " << salida << "/results.csv, ranking en " << salida
              << "/ranking.csv y rutas en " << salida << "/routes/\n";
}

// ---------------------------------------------------------------------------------------------
// Opción 4: analizar la red de recarga (RF10, RF11, RF12)
// ---------------------------------------------------------------------------------------------

void analizarRed() {
    std::cout << "\n--- ANALIZAR RED DE RECARGA ---\n";
    std::string archivo = preguntarTexto("Archivo de escenario", "data/generated/medium_1.txt");
    Escenario escenario = ArchivoEscenario::cargar(archivo);
    imprimirResumenEscenario(escenario);

    // El solver prepara el grafo de misión y la red de recarga, y clasifica a los excluidos.
    SolverMision solver(escenario);
    const GrafoMision& mision = solver.mision();
    const RedRecarga& red = solver.red();
    std::cout << std::fixed << std::setprecision(2);

    // Alcance de la red con la batería del escenario (DFS desde la base).
    std::vector<std::string> sinConexion;
    for (int s = 0; s < mision.numEstaciones(); ++s) {
        int clave = mision.claveEstacion(s);
        if (!red.esAlcanzable(clave)) sinConexion.push_back(mision.etiqueta(clave));
    }
    std::cout << "  Estaciones alcanzables desde la base: " << mision.numEstaciones() - sinConexion.size()
              << " de " << mision.numEstaciones() << '\n';
    if (!sinConexion.empty()) {
        std::cout << "  Sin conexion con la base:";
        imprimirEtiquetas(sinConexion);
    }

    // Estaciones críticas: puntos de articulación (Tarjan).
    std::vector<std::string> criticas;
    for (int clave : red.estacionesCriticas()) criticas.push_back(mision.etiqueta(clave));
    std::cout << "  Estaciones criticas (puntos de articulacion): ";
    if (criticas.empty()) {
        std::cout << "ninguna\n";
    } else {
        imprimirEtiquetas(criticas);
    }

    // Red mínima de soporte: árbol de expansión mínima entre la base y las estaciones (Prim).
    std::vector<int> nodos;
    ArbolMinimo arbol = RedRecarga::arbolSoporte(mision, nodos);
    std::cout << "  Red minima de soporte: costo " << arbol.costo << ", tramo mas largo " << arbol.aristaMaxima << '\n';
    for (size_t i = 1; i < nodos.size(); ++i) {
        if (arbol.padre[i] < 0) continue;
        int desde = nodos[arbol.padre[i]];
        std::cout << "    " << mision.etiqueta(desde) << " - " << mision.etiqueta(nodos[i])
                  << "  " << mision.distancia(desde, nodos[i]) << '\n';
    }

    // Autonomía mínima (búsqueda binaria) frente a la del escenario.
    double minima = RedRecarga::autonomiaMinima(mision);
    std::cout << "  Autonomia minima para atender a todos los clientes: " << minima
              << " (el escenario tiene " << escenario.bateria << ")\n";

    std::vector<std::string> porObstaculos, porBateria;
    for (const ClienteExcluido& cliente : solver.excluidos()) {
        if (cliente.motivo == "OBSTACLES") {
            porObstaculos.push_back(cliente.etiqueta);
        } else {
            porBateria.push_back(cliente.etiqueta);
        }
    }
    std::cout << "  Clientes separados por obstaculos: " << porObstaculos.size();
    if (!porObstaculos.empty()) imprimirEtiquetas(porObstaculos); else std::cout << '\n';
    std::cout << "  Clientes fuera del alcance de la bateria: " << porBateria.size();
    if (!porBateria.empty()) imprimirEtiquetas(porBateria); else std::cout << '\n';
}

// ---------------------------------------------------------------------------------------------

int main() {
    // I/O rápido de programación competitiva; las preguntas del menú hacen flush explícito.
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    imprimirPortada();
    while (true) {
        std::cout << "\n=== MENU ===\n"
                  << "  1. Generar datasets\n"
                  << "  2. Resolver un escenario\n"
                  << "  3. Ejecutar juez y benchmark\n"
                  << "  4. Analizar red de recarga\n"
                  << "  0. Salir\n";
        int opcion = preguntarNumero("Opcion", 0, 0, 4);
        if (opcion == 0) break;
        // Cada opción empieza con el formato normal de números: las otras dejan std::fixed activo.
        std::cout << std::defaultfloat << std::setprecision(6);
        try {
            if (opcion == 1) generarDatasets();
            if (opcion == 2) resolverEscenario();
            if (opcion == 3) ejecutarJuez();
            if (opcion == 4) analizarRed();
        } catch (const std::exception& error) {
            std::cout << "  [ERROR] " << error.what() << '\n';
        }
        memoriaUsadaEnSesion = true;
        if (entradaCerrada) break;
    }
    std::cout << "Hasta luego.\n";
    return 0;
}
