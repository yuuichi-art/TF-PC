// Las pruebas usan assert; se anula NDEBUG para que sigan activas al compilar en Release.
#undef NDEBUG

#include "../src/escenario/Escenario.hpp"
#include "../src/escenario/ArchivoEscenario.hpp"
#include "../src/geometria/Geometria.hpp"

#include <cassert>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;

// Ejemplo de docs/decisiones.md §6, copiado tal cual.
const std::string EJEMPLO_DECISIONES =
    "NAME small_001\n"
    "CATEGORY SMALL\n"
    "SEED 42\n"
    "AREA 1000 1000\n"
    "BATTERY 450\n"
    "DEPOT 500 500\n"
    "STATIONS 2\n"
    "200 300\n"
    "800 700\n"
    "CLIENTS 3\n"
    "120 880\n"
    "640 150\n"
    "900 420\n"
    "OBSTACLES 2\n"
    "4  300 400  400 400  400 500  300 500\n"
    "3  600 600  700 600  650 690\n";

// Encabezado válido de 6 líneas para armar los casos de error.
const std::string ENCABEZADO_VALIDO =
    "NAME x\nCATEGORY SMALL\nSEED 1\nAREA 100 100\nBATTERY 50\nDEPOT 0 0\n";

Escenario leerTexto(const std::string& texto) {
    std::istringstream entrada(texto);
    return ArchivoEscenario::leer(entrada);
}

bool mismosPuntos(const std::vector<Punto>& a, const std::vector<Punto>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

void verificarMismoEscenario(const Escenario& a, const Escenario& b) {
    assert(a.nombre == b.nombre);
    assert(a.categoria == b.categoria);
    assert(a.semilla == b.semilla);
    assert(a.anchoArea == b.anchoArea && a.altoArea == b.altoArea);
    assert(a.bateria == b.bateria);
    assert(a.base == b.base);
    assert(mismosPuntos(a.estaciones, b.estaciones));
    assert(mismosPuntos(a.clientes, b.clientes));
    assert(a.obstaculos.size() == b.obstaculos.size());
    for (size_t i = 0; i < a.obstaculos.size(); ++i) {
        assert(mismosPuntos(a.obstaculos[i].vertices, b.obstaculos[i].vertices));
    }
}

// Comprueba que el texto produce un error y que el mensaje contiene el fragmento esperado.
void esperarError(const std::string& texto, const std::string& fragmentoEsperado) {
    try {
        leerTexto(texto);
    } catch (const std::runtime_error& error) {
        std::string mensaje = error.what();
        if (mensaje.find(fragmentoEsperado) == std::string::npos) {
            std::cerr << "  Mensaje recibido: " << mensaje << "\n  Se esperaba: " << fragmentoEsperado << std::endl;
            assert(false);
        }
        return;
    }
    std::cerr << "  No hubo error; se esperaba: " << fragmentoEsperado << std::endl;
    assert(false);
}

void probarEjemploDecisiones() {
    std::cout << "[PRUEBA] 1. Lectura del ejemplo de docs/decisiones.md..." << std::endl;

    Escenario s = leerTexto(EJEMPLO_DECISIONES);
    assert(s.nombre == "small_001");
    assert(s.categoria == Categoria::SMALL);
    assert(s.semilla == 42);
    assert(s.anchoArea == 1000.0 && s.altoArea == 1000.0);
    assert(s.bateria == 450.0);
    assert(s.base == Punto(500, 500));
    assert(s.numEstaciones() == 2 && s.estaciones[1] == Punto(800, 700));
    assert(s.numClientes() == 3 && s.clientes[2] == Punto(900, 420));
    assert(s.numObstaculos() == 2);
    assert(s.obstaculos[0].numVertices() == 4 && s.obstaculos[1].numVertices() == 3);
    assert(s.obstaculos[1][2] == Punto(650, 690));
    assert(s.totalVerticesObstaculos() == 7);

    std::cout << "  -> OK\n" << std::endl;
}

void probarIdaYVuelta() {
    std::cout << "[PRUEBA] 2. Escribir y volver a leer conserva el escenario..." << std::endl;

    Escenario original = leerTexto(EJEMPLO_DECISIONES);
    original.bateria = 450.5;                    // Valor no entero
    original.semilla = 18446744073709551615ULL;  // Semilla máxima de 64 bits

    std::stringstream flujo;
    ArchivoEscenario::escribir(original, flujo);
    verificarMismoEscenario(original, ArchivoEscenario::leer(flujo));

    // El ejemplo escrito de nuevo debe ser idéntico al texto original.
    std::stringstream reescrito;
    ArchivoEscenario::escribir(leerTexto(EJEMPLO_DECISIONES), reescrito);
    assert(reescrito.str() == EJEMPLO_DECISIONES);

    std::cout << "  -> OK\n" << std::endl;
}

void probarGuardarYCargar() {
    std::cout << "[PRUEBA] 3. Guardar en archivo (creando carpetas) y cargar..." << std::endl;

    const std::string carpeta = "tmp_prueba_scenario";
    const std::string ruta = carpeta + "/sub/escenario.txt";
    std::filesystem::remove_all(carpeta);

    Escenario original = leerTexto(EJEMPLO_DECISIONES);
    ArchivoEscenario::guardar(original, ruta);
    verificarMismoEscenario(original, ArchivoEscenario::cargar(ruta));
    std::filesystem::remove_all(carpeta);

    bool fallo = false;
    try {
        ArchivoEscenario::cargar("no_existe.txt");
    } catch (const std::runtime_error& error) {
        fallo = std::string(error.what()).find("no se pudo abrir") != std::string::npos;
    }
    assert(fallo);

    std::cout << "  -> OK\n" << std::endl;
}

void probarErrores() {
    std::cout << "[PRUEBA] 4. Errores con linea y descripcion..." << std::endl;

    esperarError("NAME x\nCATEGORY HUGE\n", "linea 2: categoria desconocida 'HUGE'");
    esperarError("NAME x\nCATEGORY SMALL\nSEED -3\n", "linea 3: la semilla debe ser un entero no negativo");
    esperarError("NAME x\nCATEGORY SMALL\nSEED 1\nAREA 100 100\nBATTERY -5\n",
                 "linea 5: la autonomia de la bateria debe ser positiva");
    esperarError(ENCABEZADO_VALIDO + "STATION 1\n5 5\nCLIENTS 0\nOBSTACLES 0\n",
                 "linea 7: se esperaba la etiqueta STATIONS y se encontro 'STATION'");
    esperarError(ENCABEZADO_VALIDO + "STATIONS 1.5\n",
                 "linea 7: la cantidad de estaciones debe ser un entero no negativo");
    esperarError(ENCABEZADO_VALIDO + "STATIONS 1\n5 cinco\nCLIENTS 0\nOBSTACLES 0\n",
                 "linea 8: se esperaba la coordenada y de la estacion 1 (un numero) y se encontro 'cinco'");
    esperarError(ENCABEZADO_VALIDO + "STATIONS 0\nCLIENTS 2\n1 1\nOBSTACLES 0\n",
                 "linea 10: se esperaba la coordenada x del cliente 2 (un numero) y se encontro 'OBSTACLES'");
    esperarError(ENCABEZADO_VALIDO + "STATIONS 0\nCLIENTS 0\nOBSTACLES 1\n2  0 0  1 1\n",
                 "linea 10: el obstaculo 1 necesita al menos 3 vertices");
    esperarError(ENCABEZADO_VALIDO + "STATIONS 0\nCLIENTS 0\n",
                 "linea 8: el archivo termina antes de la etiqueta OBSTACLES");
    esperarError(ENCABEZADO_VALIDO + "STATIONS 0\nCLIENTS 0\nOBSTACLES 0\nextra\n",
                 "linea 10: contenido inesperado 'extra'");

    std::cout << "  -> OK\n" << std::endl;
}

// Ningún punto de interés puede estar dentro ni sobre el borde de un obstáculo.
void verificarPuntosFueraDeObstaculos(const Escenario& s) {
    std::vector<Punto> puntos = s.estaciones;
    puntos.insert(puntos.end(), s.clientes.begin(), s.clientes.end());
    puntos.push_back(s.base);
    for (const Punto& p : puntos) {
        for (const Poligono& poligono : s.obstaculos) {
            assert(Geometria::ubicacionEnPoligono(p, poligono) == UbicacionPunto::FUERA);
        }
    }
}

bool clear(const Escenario& s, const Punto& a, const Punto& b) {
    return Geometria::tramoLibre(Segmento(a, b), s.obstaculos);
}

void probarArchivosAdversariales(const std::string& carpeta) {
    std::cout << "[PRUEBA] 5. Casos adversariales de " << carpeta << "..." << std::endl;

    const std::vector<std::string> nombres = {
        "adv_collinear", "adv_enclosed", "adv_corridor", "adv_concave_trap", "adv_unreachable_battery"
    };
    for (const std::string& nombre : nombres) {
        Escenario s = ArchivoEscenario::cargar(carpeta + "/" + nombre + ".txt");
        assert(s.nombre == nombre);
        assert(s.categoria == Categoria::ADVERSARIAL);
        assert(s.numClientes() > 0);
        verificarPuntosFueraDeObstaculos(s);
    }

    // Comprobaciones geométricas de cada caso, para confirmar que miden lo que dicen.
    Escenario colineal = ArchivoEscenario::cargar(carpeta + "/adv_collinear.txt");
    assert(clear(colineal, colineal.base, colineal.clientes[0]));            // Vuela sobre el borde inferior
    assert(!clear(colineal, colineal.estaciones[0], colineal.clientes[3]));  // x = 50 cruza el obstáculo

    Escenario encerrado = ArchivoEscenario::cargar(carpeta + "/adv_enclosed.txt");
    Punto atrapado = encerrado.clientes[0];
    const std::vector<Punto> exterior = {encerrado.base, encerrado.estaciones[0], encerrado.clientes[1],
                                         encerrado.clientes[2], Punto(50, 0), Punto(50, 100),
                                         Punto(0, 50), Punto(100, 50)};
    for (const Punto& p : exterior) {
        assert(!clear(encerrado, atrapado, p));                              // Ninguna salida directa
    }

    Escenario pasillo = ArchivoEscenario::cargar(carpeta + "/adv_corridor.txt");
    assert(clear(pasillo, pasillo.base, pasillo.clientes[0]));               // Recto por el pasillo

    Escenario trampa = ArchivoEscenario::cargar(carpeta + "/adv_concave_trap.txt");
    assert(clear(trampa, trampa.base, trampa.clientes[0]));                  // Entra al hueco de la U
    assert(!clear(trampa, trampa.base, trampa.clientes[1]));                 // El cliente de atrás está bloqueado

    std::cout << "  -> OK\n" << std::endl;
}

void probarBomYNombres() {
    std::cout << "[PRUEBA] 6. Archivo con BOM UTF-8 y nombres con espacios..." << std::endl;

    // El Bloc de notas puede anteponer un BOM (EF BB BF): se ignora y el archivo se lee igual.
    Escenario conBom = leerTexto("\xEF\xBB\xBF" + EJEMPLO_DECISIONES);
    assert(conBom.nombre == "small_001" && conBom.numClientes() == 3);

    // Un nombre con espacios o vacío no se puede guardar, porque el archivo no se podría releer.
    Escenario s = leerTexto(EJEMPLO_DECISIONES);
    for (const std::string& nombreInvalido : {std::string("mi escenario"), std::string("")}) {
        s.nombre = nombreInvalido;
        std::ostringstream salida;
        bool rechazado = false;
        try {
            ArchivoEscenario::escribir(s, salida);
        } catch (const std::invalid_argument&) {
            rechazado = true;
        }
        assert(rechazado && salida.str().empty());
    }

    // guardar() valida antes de crear el archivo: no queda un archivo vacío.
    const std::string ruta = "tmp_prueba_bad_name.txt";
    std::filesystem::remove(ruta);
    s.nombre = "con espacio";
    bool rechazado = false;
    try {
        ArchivoEscenario::guardar(s, ruta);
    } catch (const std::invalid_argument&) {
        rechazado = true;
    }
    assert(rechazado && !std::filesystem::exists(ruta));

    std::cout << "  -> OK\n" << std::endl;
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // CTest pasa la carpeta de los casos adversariales como argumento.
    std::string carpetaAdversarial = argc > 1 ? argv[1] : "data/adversarial";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   GEODRONE CHALLENGE - PRUEBAS DE ESCENARIOS (TXT)      " << std::endl;
    std::cout << "=========================================================\n" << std::endl;

    probarEjemploDecisiones();
    probarIdaYVuelta();
    probarGuardarYCargar();
    probarErrores();
    probarArchivosAdversariales(carpetaAdversarial);
    probarBomYNombres();

    std::cout << "=========================================================" << std::endl;
    std::cout << "   TODAS LAS PRUEBAS DE ESCENARIOS PASARON CON EXITO     " << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
