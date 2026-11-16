#include "SolverMision.hpp"
#include "SolverExacto.hpp"
#include "SolverHeuristico.hpp"
#include "PlanificadorRecargas.hpp"

#include <algorithm>
#include <stdexcept>

namespace geodrone {

namespace {

using Reloj = std::chrono::steady_clock;

double milisegundosDesde(Reloj::time_point inicio) {
    return std::chrono::duration<double, std::milli>(Reloj::now() - inicio).count();
}

// Fracción del tiempo límite que puede usar la búsqueda local iterada, con margen para
// insertar recargas, armar la trayectoria y medir.
const double FRACCION_PRESUPUESTO = 0.9;

} // namespace

std::string nombreAlgoritmo(Algoritmo algoritmo) {
    switch (algoritmo) {
        case Algoritmo::BRANCH_AND_BOUND:       return "branch_and_bound";
        case Algoritmo::VECINO_MAS_CERCANO:       return "nearest_neighbor";
        case Algoritmo::DOS_APROXIMACION:      return "mst_approximation";
        case Algoritmo::VECINO_MAS_CERCANO_2OPT:  return "nearest_neighbor_2opt";
        case Algoritmo::DOS_APROXIMACION_2OPT: return "mst_approximation_2opt";
        case Algoritmo::BUSQUEDA_LOCAL_ITERADA:  return "iterated_local_search";
    }
    return "?";
}

const std::vector<Algoritmo>& todosLosAlgoritmos() {
    static const std::vector<Algoritmo> algoritmos = {
        Algoritmo::BRANCH_AND_BOUND, Algoritmo::VECINO_MAS_CERCANO, Algoritmo::DOS_APROXIMACION,
        Algoritmo::VECINO_MAS_CERCANO_2OPT, Algoritmo::DOS_APROXIMACION_2OPT, Algoritmo::BUSQUEDA_LOCAL_ITERADA
    };
    return algoritmos;
}

SolverMision::SolverMision(const Escenario& escenario)
    : nombre_(escenario.nombre),
      semilla_(escenario.semilla),
      limiteTiempoSegundos_(limitesDe(escenario.categoria).limiteTiempoSegundos),
      inicioConstruccion_(Reloj::now()),
      mision_(escenario),
      red_(mision_, escenario.bateria) {
    // Exclusiones (docs/decisiones.md §1): primero los que separan los obstáculos, luego los de batería.
    for (int c = 0; c < mision_.numClientes(); ++c) {
        int clave = mision_.claveCliente(c);
        if (!mision_.conectadoABase(clave)) {
            excluidos_.push_back({mision_.etiqueta(clave), "OBSTACLES"});
        } else if (!red_.puedeAtender(clave)) {
            excluidos_.push_back({mision_.etiqueta(clave), "BATTERY"});
        } else {
            atendibles_.push_back(clave);
        }
    }
    preparacionMs_ = milisegundosDesde(inicioConstruccion_);
}

bool SolverMision::admiteExacto() const {
    return static_cast<int>(atendibles_.size()) <= SolverExacto::MAX_CLIENTES;
}

std::vector<int> SolverMision::elegirOrden(Algoritmo algoritmo) const {
    const auto& distancias = mision_.distancias();
    int base = mision_.claveBase();
    switch (algoritmo) {
        case Algoritmo::BRANCH_AND_BOUND:
            return SolverExacto::resolver(distancias, base, atendibles_).orden;
        case Algoritmo::VECINO_MAS_CERCANO:
            return SolverHeuristico::vecinoMasCercano(distancias, base, atendibles_);
        case Algoritmo::DOS_APROXIMACION:
            return SolverHeuristico::dosAproximacion(distancias, base, atendibles_);
        case Algoritmo::VECINO_MAS_CERCANO_2OPT: {
            std::vector<int> orden = SolverHeuristico::vecinoMasCercano(distancias, base, atendibles_);
            SolverHeuristico::aplicar2Opt(distancias, base, orden);
            return orden;
        }
        case Algoritmo::DOS_APROXIMACION_2OPT: {
            std::vector<int> orden = SolverHeuristico::dosAproximacion(distancias, base, atendibles_);
            SolverHeuristico::aplicar2Opt(distancias, base, orden);
            return orden;
        }
        case Algoritmo::BUSQUEDA_LOCAL_ITERADA: {
            // Presupuesto: el 90 % del límite de la categoría menos lo que ya costó la preparación.
            double presupuestoMs = std::max(0.0, FRACCION_PRESUPUESTO * limiteTiempoSegundos_ * 1000.0 - preparacionMs_);
            auto limite = Reloj::now() + std::chrono::microseconds(static_cast<long long>(presupuestoMs * 1000.0));
            std::vector<int> inicio = SolverHeuristico::vecinoMasCercano(distancias, base, atendibles_);
            return SolverHeuristico::busquedaLocalIterada(distancias, base, inicio, semilla_,
                                                        SolverHeuristico::ITERACIONES_POR_DEFECTO, limite).orden;
        }
    }
    return {};
}

// Cada tramo entre nodos clave se reemplaza por su camino real en el grafo de visibilidad.
std::vector<PuntoRuta> SolverMision::expandirTrayectoria(const std::vector<int>& secuenciaClaves) const {
    const GrafoVisibilidad& grafo = mision_.grafo();
    auto aPuntoRuta = [&](int nodo) {
        const Nodo& n = grafo.nodo(nodo);
        return PuntoRuta{n.punto, n.esNodoClave() ? n.nombre : "V"};
    };

    std::vector<PuntoRuta> trayectoria;
    if (secuenciaClaves.empty()) return trayectoria;
    trayectoria.push_back(aPuntoRuta(secuenciaClaves.front()));
    for (size_t i = 0; i + 1 < secuenciaClaves.size(); ++i) {
        std::vector<int> tramo = mision_.nodosCamino(secuenciaClaves[i], secuenciaClaves[i + 1]);
        for (size_t j = 1; j < tramo.size(); ++j) {   // El primer nodo ya está en el camino
            trayectoria.push_back(aPuntoRuta(tramo[j]));
        }
    }
    return trayectoria;
}

Ruta SolverMision::resolver(Algoritmo algoritmo) const {
    if (algoritmo == Algoritmo::BRANCH_AND_BOUND && !admiteExacto()) {
        throw std::invalid_argument("el solver exacto admite hasta " + std::to_string(SolverExacto::MAX_CLIENTES) +
                                    " clientes y este escenario tiene " + std::to_string(atendibles_.size()));
    }
    auto inicio = Reloj::now();

    Ruta ruta;
    ruta.escenario = nombre_;
    ruta.algoritmo = nombreAlgoritmo(algoritmo);
    ruta.excluidos = excluidos_;
    ruta.preparacionMs = preparacionMs_;

    PlanRecargas plan = PlanificadorRecargas(mision_, red_).planificar(elegirOrden(algoritmo));
    ruta.factible = plan.factible;
    ruta.costo = plan.costo;
    ruta.recargas = plan.recargas;
    for (int clave : plan.secuenciaClaves) {
        ruta.orden.push_back(mision_.etiqueta(clave));
    }
    ruta.trayectoria = expandirTrayectoria(plan.secuenciaClaves);

    ruta.resolucionMs = milisegundosDesde(inicio);
    return ruta;
}

} // namespace geodrone
