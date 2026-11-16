#include "ArchivoEscenario.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace geodrone {
namespace ArchivoEscenario {

namespace {

// Valor leído del archivo junto con la línea donde aparece, para dar errores útiles.
struct Palabra {
    std::string texto;
    int linea;
};

// Lector secuencial de valores. El formato es una lista fija de etiquetas y números,
// así que basta con consumirlos en orden y validar cada uno al leerlo.
class LectorPalabras {
public:
    explicit LectorPalabras(std::istream& entrada) {
        std::string linea;
        int numeroLinea = 0;
        while (std::getline(entrada, linea)) {
            ++numeroLinea;
            // El Bloc de notas puede guardar "UTF-8 con BOM": tres bytes invisibles al inicio
            // que, si no se quitan, se pegan a la primera etiqueta.
            if (numeroLinea == 1 && linea.compare(0, 3, "\xEF\xBB\xBF") == 0) {
                linea.erase(0, 3);
            }
            std::istringstream palabras(linea);
            std::string palabra;
            while (palabras >> palabra) {
                palabras_.push_back({palabra, numeroLinea});
            }
        }
    }

    void esperarEtiqueta(const std::string& etiqueta) {
        const Palabra& palabra = siguiente("la etiqueta " + etiqueta);
        if (palabra.texto != etiqueta) {
            fallarEn(palabra.linea, "se esperaba la etiqueta " + etiqueta + " y se encontro '" + palabra.texto + "'");
        }
    }

    std::string leerPalabra(const std::string& esperado) {
        return siguiente(esperado).texto;
    }

    double leerNumero(const std::string& esperado) {
        const Palabra& palabra = siguiente(esperado);
        double valor = 0.0;
        if (!interpretarReal(palabra.texto, valor)) {
            fallarEn(palabra.linea, "se esperaba " + esperado + " (un numero) y se encontro '" + palabra.texto + "'");
        }
        return valor;
    }

    // Cantidad de elementos de un bloque: entero no negativo.
    int leerCantidad(const std::string& esperado) {
        double valor = leerNumero(esperado);
        if (valor < 0 || valor != std::floor(valor) || valor > 1e7) {
            fallarEnUltima(esperado + " debe ser un entero no negativo");
        }
        return static_cast<int>(valor);
    }

    uint64_t leerSemilla() {
        const Palabra& palabra = siguiente("la semilla");
        bool valido = !palabra.texto.empty() && palabra.texto.find_first_not_of("0123456789") == std::string::npos;
        if (!valido) {
            fallarEn(palabra.linea, "la semilla debe ser un entero no negativo y se encontro '" + palabra.texto + "'");
        }
        try {
            return std::stoull(palabra.texto);
        } catch (const std::exception&) {
            fallarEn(palabra.linea, "la semilla '" + palabra.texto + "' es demasiado grande");
        }
    }

    // deQue completa el mensaje de error: "de la base", "del cliente 2", ...
    Punto leerPunto(const std::string& deQue) {
        double x = leerNumero("la coordenada x " + deQue);
        double y = leerNumero("la coordenada y " + deQue);
        return Punto(x, y);
    }

    // Tras los obstáculos no debe quedar nada: así se detectan cantidades mal escritas.
    void esperarFin() {
        if (posicion_ < palabras_.size()) {
            fallarEn(palabras_[posicion_].linea,
                     "contenido inesperado '" + palabras_[posicion_].texto + "' despues de los obstaculos");
        }
    }

    [[noreturn]] void fallarEnUltima(const std::string& mensaje) const {
        fallarEn(posicion_ == 0 ? 0 : palabras_[posicion_ - 1].linea, mensaje);
    }

private:
    std::vector<Palabra> palabras_;
    size_t posicion_ = 0;

    const Palabra& siguiente(const std::string& esperado) {
        if (posicion_ >= palabras_.size()) {
            int ultimaLinea = palabras_.empty() ? 0 : palabras_.back().linea;
            fallarEn(ultimaLinea, "el archivo termina antes de " + esperado);
        }
        return palabras_[posicion_++];
    }

    static bool interpretarReal(const std::string& texto, double& valor) {
        try {
            size_t usados = 0;
            valor = std::stod(texto, &usados);
            return usados == texto.size() && std::isfinite(valor);
        } catch (const std::exception&) {
            return false;
        }
    }

    [[noreturn]] static void fallarEn(int linea, const std::string& mensaje) {
        throw std::runtime_error("linea " + std::to_string(linea) + ": " + mensaje);
    }
};

// Bloque "ETIQUETA cantidad" seguido de un punto por elemento (estaciones o clientes).
// nombrePlural y delElemento solo sirven para los mensajes de error ("clientes", "del cliente").
std::vector<Punto> leerBloquePuntos(LectorPalabras& lector, const std::string& etiqueta,
                                    const std::string& nombrePlural, const std::string& delElemento) {
    lector.esperarEtiqueta(etiqueta);
    int cantidad = lector.leerCantidad("la cantidad de " + nombrePlural);
    std::vector<Punto> puntos;
    puntos.reserve(cantidad);
    for (int i = 0; i < cantidad; ++i) {
        puntos.push_back(lector.leerPunto(delElemento + " " + std::to_string(i + 1)));
    }
    return puntos;
}

// El nombre se lee como una sola palabra: con espacios el archivo no se podría volver a leer.
void validarNombre(const std::string& nombre) {
    bool valido = !nombre.empty() &&
                 std::none_of(nombre.begin(), nombre.end(), [](unsigned char c) { return std::isspace(c) != 0; });
    if (!valido) {
        throw std::invalid_argument("el nombre del escenario no puede estar vacio ni tener espacios: '" + nombre + "'");
    }
}

std::vector<Poligono> leerObstaculos(LectorPalabras& lector) {
    lector.esperarEtiqueta("OBSTACLES");
    int cantidad = lector.leerCantidad("la cantidad de obstaculos");
    std::vector<Poligono> obstaculos;
    obstaculos.reserve(cantidad);
    for (int i = 0; i < cantidad; ++i) {
        std::string numero = std::to_string(i + 1);
        int cantidadVertices = lector.leerCantidad("la cantidad de vertices del obstaculo " + numero);
        if (cantidadVertices < 3) {
            lector.fallarEnUltima("el obstaculo " + numero + " necesita al menos 3 vertices");
        }
        std::vector<Punto> vertices;
        vertices.reserve(cantidadVertices);
        for (int v = 0; v < cantidadVertices; ++v) {
            vertices.push_back(lector.leerPunto("del vertice " + std::to_string(v + 1) + " del obstaculo " + numero));
        }
        obstaculos.emplace_back(i + 1, vertices, "Obs" + std::to_string(i + 1));
    }
    return obstaculos;
}

} // namespace

Escenario leer(std::istream& entrada) {
    LectorPalabras lector(entrada);
    Escenario escenario;

    lector.esperarEtiqueta("NAME");
    escenario.nombre = lector.leerPalabra("el nombre del escenario");

    lector.esperarEtiqueta("CATEGORY");
    std::string textoCategoria = lector.leerPalabra("la categoria");
    if (!interpretarCategoria(textoCategoria, escenario.categoria)) {
        lector.fallarEnUltima("categoria desconocida '" + textoCategoria +
                              "' (se espera SMALL, MEDIUM, LARGE, EXTREME o ADVERSARIAL)");
    }

    lector.esperarEtiqueta("SEED");
    escenario.semilla = lector.leerSemilla();

    lector.esperarEtiqueta("AREA");
    escenario.anchoArea = lector.leerNumero("el ancho del area");
    escenario.altoArea = lector.leerNumero("el alto del area");
    if (escenario.anchoArea <= 0 || escenario.altoArea <= 0) {
        lector.fallarEnUltima("las dimensiones del area deben ser positivas");
    }

    lector.esperarEtiqueta("BATTERY");
    escenario.bateria = lector.leerNumero("la autonomia de la bateria");
    if (escenario.bateria <= 0) {
        lector.fallarEnUltima("la autonomia de la bateria debe ser positiva");
    }

    lector.esperarEtiqueta("DEPOT");
    escenario.base = lector.leerPunto("de la base");

    escenario.estaciones = leerBloquePuntos(lector, "STATIONS", "estaciones", "de la estacion");
    escenario.clientes = leerBloquePuntos(lector, "CLIENTS", "clientes", "del cliente");
    escenario.obstaculos = leerObstaculos(lector);
    lector.esperarFin();

    return escenario;
}

Escenario cargar(const std::string& ruta) {
    std::ifstream archivo(ruta);
    if (!archivo) {
        throw std::runtime_error("no se pudo abrir el archivo " + ruta);
    }
    try {
        return leer(archivo);
    } catch (const std::runtime_error& error) {
        throw std::runtime_error(ruta + ", " + error.what());
    }
}

void escribir(const Escenario& escenario, std::ostream& salida) {
    // 15 cifras significativas: los enteros se escriben sin decimales y los reales sin pérdida visible.
    validarNombre(escenario.nombre);

    salida << std::setprecision(15);
    salida << "NAME " << escenario.nombre << '\n';
    salida << "CATEGORY " << nombreCategoria(escenario.categoria) << '\n';
    salida << "SEED " << escenario.semilla << '\n';
    salida << "AREA " << escenario.anchoArea << ' ' << escenario.altoArea << '\n';
    salida << "BATTERY " << escenario.bateria << '\n';
    salida << "DEPOT " << escenario.base.x << ' ' << escenario.base.y << '\n';

    salida << "STATIONS " << escenario.estaciones.size() << '\n';
    for (const Punto& p : escenario.estaciones) {
        salida << p.x << ' ' << p.y << '\n';
    }

    salida << "CLIENTS " << escenario.clientes.size() << '\n';
    for (const Punto& p : escenario.clientes) {
        salida << p.x << ' ' << p.y << '\n';
    }

    salida << "OBSTACLES " << escenario.obstaculos.size() << '\n';
    for (const Poligono& poligono : escenario.obstaculos) {
        salida << poligono.numVertices();
        for (const Punto& v : poligono.vertices) {
            salida << "  " << v.x << ' ' << v.y;
        }
        salida << '\n';
    }
}

std::ofstream abrirParaEscribir(const std::string& ruta) {
    std::filesystem::path rutaArchivo(ruta);
    if (rutaArchivo.has_parent_path()) {
        std::filesystem::create_directories(rutaArchivo.parent_path());
    }
    std::ofstream archivo(ruta);
    if (!archivo) {
        throw std::runtime_error("no se pudo crear el archivo " + ruta);
    }
    return archivo;
}

void guardar(const Escenario& escenario, const std::string& ruta) {
    validarNombre(escenario.nombre);   // Antes de crear el archivo, para no dejarlo vacío
    std::ofstream archivo = abrirParaEscribir(ruta);
    escribir(escenario, archivo);
    if (!archivo) {
        throw std::runtime_error("error al escribir el archivo " + ruta);
    }
}

} // namespace ArchivoEscenario
} // namespace geodrone
