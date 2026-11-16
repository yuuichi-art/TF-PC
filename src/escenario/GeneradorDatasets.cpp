#include "GeneradorDatasets.hpp"
#include "Aleatorio.hpp"
#include "ArchivoEscenario.hpp"
#include "../geometria/EnvolventeConvexa.hpp"
#include "../geometria/Geometria.hpp"
#include "../grafo/RedRecarga.hpp"
#include "../grafo/GrafoMision.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <utility>

namespace geodrone {

namespace {

// Obstáculos convexos sin solape. El área se divide en una cuadrícula de lado x lado celdas
// (unas el doble que obstáculos, para dejar espacio libre) y cada obstáculo ocupa una celda
// distinta elegida al azar. Dentro de la celda se deja un margen del 15 % por lado, así que
// dos obstáculos nunca se tocan.
std::vector<Poligono> generarObstaculos(std::mt19937_64& rng, int cantidad, int maxPuntos,
                                       double ancho, double alto) {
    std::vector<Poligono> obstaculos;
    if (cantidad == 0) return obstaculos;

    int lado = std::max(3, static_cast<int>(std::ceil(std::sqrt(2.0 * cantidad))));
    double anchoCelda = ancho / lado;
    double altoCelda = alto / lado;

    // Orden aleatorio de las celdas (Fisher-Yates).
    std::vector<int> celdas(lado * lado);
    std::iota(celdas.begin(), celdas.end(), 0);
    for (int i = static_cast<int>(celdas.size()) - 1; i > 0; --i) {
        std::swap(celdas[i], celdas[enteroAleatorio(rng, 0, i)]);
    }

    obstaculos.reserve(cantidad);
    for (int k = 0; k < cantidad; ++k) {
        int columna = celdas[k] % lado;
        int fila = celdas[k] / lado;
        long long minX = static_cast<long long>(std::ceil((columna + 0.15) * anchoCelda));
        long long maxX = static_cast<long long>(std::floor((columna + 0.85) * anchoCelda));
        long long minY = static_cast<long long>(std::ceil((fila + 0.15) * altoCelda));
        long long maxY = static_cast<long long>(std::floor((fila + 0.85) * altoCelda));

        // Se repite en el caso raro de puntos colineales (envolvente con menos de 3 vértices).
        std::vector<Punto> envolvente;
        while (envolvente.size() < 3) {
            int cantidadPuntos = static_cast<int>(enteroAleatorio(rng, 3, maxPuntos));
            std::vector<Punto> puntos;
            puntos.reserve(cantidadPuntos);
            for (int p = 0; p < cantidadPuntos; ++p) {
                puntos.emplace_back(static_cast<double>(enteroAleatorio(rng, minX, maxX)),
                                    static_cast<double>(enteroAleatorio(rng, minY, maxY)));
            }
            envolvente = envolventeConvexa(puntos);
        }
        obstaculos.emplace_back(k + 1, envolvente, "Obs" + std::to_string(k + 1));
    }
    return obstaculos;
}

// Punto entero libre: fuera de todo obstáculo (ni dentro ni sobre el borde) y no usado antes.
Punto puntoLibreAleatorio(std::mt19937_64& rng, double ancho, double alto,
                      const std::vector<Poligono>& obstaculos,
                      std::set<std::pair<long long, long long>>& usados) {
    while (true) {
        long long x = enteroAleatorio(rng, 0, static_cast<long long>(ancho));
        long long y = enteroAleatorio(rng, 0, static_cast<long long>(alto));
        if (usados.count({x, y})) continue;

        Punto candidato(static_cast<double>(x), static_cast<double>(y));
        bool libre = true;
        for (const Poligono& poligono : obstaculos) {
            if (Geometria::ubicacionEnPoligono(candidato, poligono) != UbicacionPunto::FUERA) {
                libre = false;
                break;
            }
        }
        if (libre) {
            usados.insert({x, y});
            return candidato;
        }
    }
}

std::string minusculas(std::string texto) {
    for (char& c : texto) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return texto;
}

} // namespace

RangosCategoria GeneradorDatasets::rangosDe(Categoria categoria) {
    // Clientes y obstáculos según la tabla del README. Puntos por obstáculo elegidos para
    // respetar el máximo de vértices de obstáculos de cada categoría (30, 100, 400 y 1500).
    switch (categoria) {
        case Categoria::SMALL:   return {1, 10, 1, 3, 1, 5, 6};
        case Categoria::MEDIUM:  return {11, 40, 3, 6, 5, 15, 6};
        case Categoria::LARGE:   return {41, 200, 6, 12, 15, 50, 8};
        case Categoria::EXTREME: return {201, 1000, 12, 25, 50, 150, 10};
        case Categoria::ADVERSARIAL: break;
    }
    throw std::invalid_argument("la categoria ADVERSARIAL no se genera: sus casos se escriben a mano");
}

RangosCategoria GeneradorDatasets::rangosPeorCaso(Categoria categoria) {
    RangosCategoria rangos = rangosDe(categoria);
    rangos.minClientes = rangos.maxClientes;
    rangos.minEstaciones = rangos.maxEstaciones;
    rangos.minObstaculos = rangos.maxObstaculos;
    return rangos;
}

Escenario GeneradorDatasets::generar(Categoria categoria, uint64_t semilla, double ancho, double alto) {
    Escenario escenario = generarGeometria(rangosDe(categoria), categoria, semilla, ancho, alto);
    escenario.bateria = bateriaPara(escenario);
    return escenario;
}

double GeneradorDatasets::bateriaPara(const Escenario& geometria) {
    GrafoMision mision(geometria);
    double autonomia = RedRecarga::autonomiaMinima(mision);
    return std::max(1.0, std::ceil(autonomia * MARGEN_BATERIA));
}

Escenario GeneradorDatasets::generarGeometria(const RangosCategoria& rangos, Categoria categoria, uint64_t semilla,
                                          double ancho, double alto) {
    std::mt19937_64 rng(semilla);

    Escenario escenario;
    escenario.categoria = categoria;
    escenario.semilla = semilla;
    escenario.nombre = minusculas(nombreCategoria(categoria)) + "_" + std::to_string(semilla);
    escenario.anchoArea = ancho;
    escenario.altoArea = alto;
    escenario.bateria = 0.0;   // Sin calcular: lo completa bateriaPara

    // El orden de los sorteos es fijo: cambiarlo cambia los escenarios de cada semilla.
    int cantidadClientes = static_cast<int>(enteroAleatorio(rng, rangos.minClientes, rangos.maxClientes));
    int cantidadEstaciones = static_cast<int>(enteroAleatorio(rng, rangos.minEstaciones, rangos.maxEstaciones));
    int cantidadObstaculos = static_cast<int>(enteroAleatorio(rng, rangos.minObstaculos, rangos.maxObstaculos));

    escenario.obstaculos = generarObstaculos(rng, cantidadObstaculos, rangos.maxPuntosPorObstaculo, ancho, alto);

    std::set<std::pair<long long, long long>> usados;
    escenario.base = puntoLibreAleatorio(rng, ancho, alto, escenario.obstaculos, usados);
    for (int i = 0; i < cantidadEstaciones; ++i) {
        escenario.estaciones.push_back(puntoLibreAleatorio(rng, ancho, alto, escenario.obstaculos, usados));
    }
    for (int i = 0; i < cantidadClientes; ++i) {
        escenario.clientes.push_back(puntoLibreAleatorio(rng, ancho, alto, escenario.obstaculos, usados));
    }
    return escenario;
}

std::vector<std::string> GeneradorDatasets::generarConjuntoBenchmark(const std::string& carpeta, int semillasPorCategoria,
                                                                bool incluirPeorCaso) {
    const Categoria categorias[] = {
        Categoria::SMALL, Categoria::MEDIUM, Categoria::LARGE, Categoria::EXTREME
    };
    std::vector<std::string> rutas;
    auto guardarEscenario = [&](const Escenario& escenario) {
        std::string ruta = carpeta + "/" + escenario.nombre + ".txt";
        ArchivoEscenario::guardar(escenario, ruta);
        rutas.push_back(ruta);
    };

    for (Categoria categoria : categorias) {
        for (int semilla = 1; semilla <= semillasPorCategoria; ++semilla) {
            guardarEscenario(generar(categoria, static_cast<uint64_t>(semilla)));
        }
    }
    if (incluirPeorCaso) {
        Escenario peor = generarGeometria(rangosPeorCaso(Categoria::EXTREME), Categoria::EXTREME, 1);
        peor.nombre = "extreme_max";
        peor.bateria = bateriaPara(peor);
        guardarEscenario(peor);
    }
    return rutas;
}

} // namespace geodrone
