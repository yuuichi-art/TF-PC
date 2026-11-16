#include "Juez.hpp"
#include "UsoMemoria.hpp"
#include "../geometria/Geometria.hpp"
#include "../grafo/ArbolMinimo.hpp"
#include "../exportacion/Exportador.hpp"
#include "../escenario/ArchivoEscenario.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>

namespace geodrone {
namespace Juez {

namespace {

const double TOL = RedRecarga::TOLERANCIA_BATERIA;
const double NO_DISPONIBLE = std::numeric_limits<double>::quiet_NaN();   // Gap de una fila que no es OK

ResultadoVerificacion falla(const std::string& problema) {
    return {false, problema};
}

// Interpreta una etiqueta "D0", "S2" o "C5": tipo y número (1..S o 1..N; 0 para la base).
// Devuelve false si no corresponde a ningún punto del escenario.
bool interpretarEtiqueta(const Escenario& s, const std::string& etiqueta, char& tipo, int& numero) {
    if (etiqueta.size() < 2) return false;
    tipo = etiqueta[0];
    try {
        size_t usados = 0;
        numero = std::stoi(etiqueta.substr(1), &usados);
        if (usados != etiqueta.size() - 1) return false;
    } catch (const std::exception&) {
        return false;
    }
    if (tipo == 'D') return numero == 0;
    if (tipo == 'S') return numero >= 1 && numero <= s.numEstaciones();
    if (tipo == 'C') return numero >= 1 && numero <= s.numClientes();
    return false;
}

// Coordenadas que el escenario asigna a la etiqueta (ya validada con interpretarEtiqueta).
Punto puntoDe(const Escenario& s, char tipo, int numero) {
    if (tipo == 'D') return s.base;
    if (tipo == 'S') return s.estaciones[numero - 1];
    return s.clientes[numero - 1];
}

double porcentajeSobre(double valor, double referencia) {
    return referencia > 0 ? 100.0 * (valor / referencia - 1.0) : 0.0;
}

// Una exclusión es real si el grafo de misión la confirma.
bool exclusionEsReal(const SolverMision& solver, int indiceCliente, const std::string& motivo) {
    int clave = solver.mision().claveCliente(indiceCliente);
    bool conectado = solver.mision().conectadoABase(clave);
    if (motivo == "OBSTACLES") return !conectado;
    if (motivo == "BATTERY") return conectado && !solver.red().puedeAtender(clave);
    return false;
}

// Tamaño de un escenario para ordenarlos de menor a mayor.
int tamanoDe(const Escenario& s) {
    return s.numClientes() + s.numEstaciones() + s.totalVerticesObstaculos();
}

} // namespace

ResultadoVerificacion verificarRuta(const Escenario& escenario, const SolverMision& solver, const Ruta& ruta) {
    if (!ruta.factible) return falla("el solver no encontro una ruta factible");
    if (ruta.trayectoria.empty()) return falla("la trayectoria esta vacia");
    if (ruta.trayectoria.front().etiqueta != "D0" || ruta.trayectoria.back().etiqueta != "D0") {
        return falla("la ruta no empieza y termina en la base");
    }

    double bateria = escenario.bateria;
    double longitud = 0.0;
    int recargas = 0;
    std::vector<int> visitas(escenario.numClientes(), 0);
    std::vector<std::string> etiquetasClave;     // Nodos clave de la trayectoria, para compararlos con ORDER

    for (size_t i = 0; i < ruta.trayectoria.size(); ++i) {
        const PuntoRuta& p = ruta.trayectoria[i];
        char tipo = 'V';
        int numero = 0;
        if (p.etiqueta != "V") {
            if (!interpretarEtiqueta(escenario, p.etiqueta, tipo, numero)) {
                return falla("etiqueta desconocida '" + p.etiqueta + "'");
            }
            if (p.punto != puntoDe(escenario, tipo, numero)) {
                return falla(p.etiqueta + " no esta en las coordenadas del escenario");
            }
            etiquetasClave.push_back(p.etiqueta);
        }
        if (i == 0) continue;

        Segmento tramo(ruta.trayectoria[i - 1].punto, p.punto);
        if (!Geometria::tramoLibre(tramo, escenario.obstaculos)) {
            return falla("el tramo " + std::to_string(i) + " (hacia " + p.etiqueta + ") cruza un obstaculo");
        }
        longitud += tramo.longitud();
        bateria -= tramo.longitud();
        if (bateria < -TOL) {
            return falla("la bateria se agota en el tramo " + std::to_string(i) + " (hacia " + p.etiqueta + ")");
        }

        // Llegar a la base o a una estación llena la batería. La llegada final a la base no
        // cuenta como recarga, porque ahí termina la misión.
        if (tipo == 'D' || tipo == 'S') {
            bateria = escenario.bateria;
            if (i + 1 < ruta.trayectoria.size()) ++recargas;
        } else if (tipo == 'C' && ++visitas[numero - 1] > 1) {
            return falla(p.etiqueta + " se visita mas de una vez");
        }
    }

    std::vector<char> excluidos(escenario.numClientes(), 0);
    for (const ClienteExcluido& cliente : ruta.excluidos) {
        char tipo = 'V';
        int numero = 0;
        if (!interpretarEtiqueta(escenario, cliente.etiqueta, tipo, numero) || tipo != 'C') {
            return falla("excluido desconocido '" + cliente.etiqueta + "'");
        }
        if (visitas[numero - 1] > 0) return falla(cliente.etiqueta + " esta excluido y tambien se visita");
        if (!exclusionEsReal(solver, numero - 1, cliente.motivo)) {
            return falla(cliente.etiqueta + " se excluyo por " + cliente.motivo + " pero se puede atender");
        }
        excluidos[numero - 1] = 1;
    }
    for (int c = 0; c < escenario.numClientes(); ++c) {
        if (visitas[c] == 0 && !excluidos[c]) {
            return falla("C" + std::to_string(c + 1) + " no se visita ni esta excluido");
        }
    }

    if (std::abs(longitud - ruta.costo) > TOL * std::max(1.0, longitud)) {
        return falla("el costo informado no coincide con la trayectoria");
    }
    if (recargas != ruta.recargas) return falla("las recargas informadas no coinciden con la trayectoria");
    if (etiquetasClave != ruta.orden) return falla("el orden informado no coincide con la trayectoria");
    return {};
}

std::string veredictoPara(bool valida, double tiempoMs, double memoriaMB, const LimitesCategoria& limites) {
    if (!valida) return "WA";
    if (tiempoMs > limites.limiteTiempoSegundos * 1000.0) return "TLE";
    if (memoriaMB > limites.limiteMemoriaMB) return "MLE";
    return "OK";
}

std::vector<std::string> buscarEscenarios(const std::string& carpeta) {
    std::vector<std::string> archivos;
    if (!std::filesystem::is_directory(carpeta)) return archivos;
    for (const auto& entrada : std::filesystem::directory_iterator(carpeta)) {
        if (entrada.is_regular_file() && entrada.path().extension() == ".txt") {
            archivos.push_back(entrada.path().generic_string());
        }
    }
    std::sort(archivos.begin(), archivos.end());
    return archivos;
}

namespace {

// Resuelve un escenario con cada algoritmo, guarda y verifica cada ruta, y le da veredicto y gap.
std::vector<FilaJuez> juzgarEscenario(const Escenario& s, const std::string& carpetaRutas) {
    SolverMision solver(s);
    bool casoPequeno = s.categoria == Categoria::SMALL || s.categoria == Categoria::ADVERSARIAL;

    std::vector<FilaJuez> filas;
    std::vector<bool> validas;
    for (Algoritmo algoritmo : todosLosAlgoritmos()) {
        if (algoritmo == Algoritmo::BRANCH_AND_BOUND && !(casoPequeno && solver.admiteExacto())) continue;
        Ruta ruta = solver.resolver(algoritmo);
        Exportador::guardarRuta(ruta, carpetaRutas + "/" + ruta.escenario + "_" + ruta.algoritmo + ".txt");
        ResultadoVerificacion verificacion = verificarRuta(s, solver, ruta);

        FilaJuez fila;
        fila.escenario = s.nombre;
        fila.categoria = s.categoria;
        fila.clientes = s.numClientes();
        fila.algoritmo = ruta.algoritmo;
        fila.problema = verificacion.problema;
        fila.costo = ruta.costo;
        fila.recargas = ruta.recargas;
        fila.excluidos = static_cast<int>(ruta.excluidos.size());
        fila.tiempoMs = ruta.totalMs();
        filas.push_back(fila);
        validas.push_back(verificacion.valida);
    }

    // Memoria pico al terminar el caso; los veredictos se dan después de medirla.
    double memoria = memoriaPicoMB();
    LimitesCategoria limites = limitesDe(s.categoria);
    double mejor = INF;
    for (size_t i = 0; i < filas.size(); ++i) {
        filas[i].memoriaMB = memoria;
        filas[i].veredicto = veredictoPara(validas[i], filas[i].tiempoMs, memoria, limites);
        if (filas[i].veredicto == "OK") mejor = std::min(mejor, filas[i].costo);
    }

    // Gap contra la mejor ruta OK y contra la cota inferior: el árbol de expansión mínima de la
    // base y los clientes atendibles.
    std::vector<int> nodos = {solver.mision().claveBase()};
    nodos.insert(nodos.end(), solver.clientesAtendibles().begin(), solver.clientesAtendibles().end());
    double cota = arbolMinimoPrim(solver.mision().distancias(), nodos).costo;
    for (FilaJuez& fila : filas) {
        bool esOk = fila.veredicto == "OK";
        fila.gapMejor = esOk ? porcentajeSobre(fila.costo, mejor) : NO_DISPONIBLE;
        fila.gapCota = esOk ? porcentajeSobre(fila.costo, cota) : NO_DISPONIBLE;
    }
    return filas;
}

// Avance en consola: una línea por caso y el motivo de cada ruta que no dio OK.
void imprimirCaso(std::ostream& registro, size_t numero, size_t total, const Escenario& s,
                  const std::vector<FilaJuez>& filas) {
    int ok = 0;
    double mejor = INF;
    for (const FilaJuez& fila : filas) {
        if (fila.veredicto == "OK") {
            ++ok;
            mejor = std::min(mejor, fila.costo);
        }
    }
    registro << "  [" << std::setw(2) << numero << "/" << total << "] " << std::left << std::setw(26) << s.nombre
        << std::right << std::setw(5) << s.numClientes() << " clientes  " << ok << "/" << filas.size()
        << " OK  mejor " << std::fixed << std::setprecision(2) << mejor << "\n" << std::flush;
    for (const FilaJuez& fila : filas) {
        if (fila.veredicto != "OK") {
            registro << "         " << fila.algoritmo << ": " << fila.veredicto
                << (fila.problema.empty() ? "" : " (" + fila.problema + ")") << "\n";
        }
    }
}

} // namespace

std::vector<FilaJuez> ejecutar(const std::vector<std::string>& archivosEscenario, const std::string& carpetaRutas,
                               std::ostream& registro) {
    std::vector<Escenario> escenarios;
    for (const std::string& archivo : archivosEscenario) escenarios.push_back(ArchivoEscenario::cargar(archivo));
    std::stable_sort(escenarios.begin(), escenarios.end(),
                     [](const Escenario& a, const Escenario& b) { return tamanoDe(a) < tamanoDe(b); });

    std::vector<FilaJuez> filas;
    for (size_t i = 0; i < escenarios.size(); ++i) {
        std::vector<FilaJuez> filasCaso = juzgarEscenario(escenarios[i], carpetaRutas);
        imprimirCaso(registro, i + 1, escenarios.size(), escenarios[i], filasCaso);
        filas.insert(filas.end(), filasCaso.begin(), filasCaso.end());
    }
    return filas;
}

std::vector<FilaRanking> armarRanking(const std::vector<FilaJuez>& filas) {
    // Acumulados por categoría y algoritmo.
    std::map<std::pair<int, std::string>, FilaRanking> totales;
    for (const FilaJuez& fila : filas) {
        FilaRanking& total = totales[{static_cast<int>(fila.categoria), fila.algoritmo}];
        total.categoria = fila.categoria;
        total.algoritmo = fila.algoritmo;
        ++total.casos;
        if (fila.veredicto == "OK") {
            ++total.casosOk;
            total.gapMedio += fila.gapMejor;
            total.tiempoMedioMs += fila.tiempoMs;
        }
    }

    // Promedios sobre los casos OK, agrupados por categoría para ordenarlos dentro de cada una.
    std::map<int, std::vector<FilaRanking>> porCategoria;
    for (auto& entrada : totales) {
        FilaRanking& total = entrada.second;
        if (total.casosOk > 0) {
            total.gapMedio /= total.casosOk;
            total.tiempoMedioMs /= total.casosOk;
        }
        porCategoria[entrada.first.first].push_back(total);
    }

    std::vector<FilaRanking> ranking;
    for (auto& entrada : porCategoria) {
        std::vector<FilaRanking>& lista = entrada.second;
        std::sort(lista.begin(), lista.end(), [](const FilaRanking& a, const FilaRanking& b) {
            if (a.casosOk != b.casosOk) return a.casosOk > b.casosOk;
            if (a.gapMedio != b.gapMedio) return a.gapMedio < b.gapMedio;
            return a.tiempoMedioMs < b.tiempoMedioMs;
        });
        for (size_t i = 0; i < lista.size(); ++i) {
            lista[i].posicion = static_cast<int>(i) + 1;
            ranking.push_back(lista[i]);
        }
    }
    return ranking;
}

} // namespace Juez
} // namespace geodrone
