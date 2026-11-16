cmake_minimum_required(VERSION 3.16)

project(GeoDroneChallenge
    VERSION 1.0.0
    DESCRIPTION "Optimizacion de Rutas de Entrega con Drones en Entornos Urbanos - USIL 2026"
    LANGUAGES CXX
)

# Estándar de C++17, sin extensiones del compilador
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# La optimización depende del tipo de compilación: Release usa -O3 -DNDEBUG (/O2 en MSVC)
# y Debug compila sin optimizar, para poder depurar. Si no se indica tipo, se usa Release.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Tipo de compilacion" FORCE)
endif()

# Advertencias y opciones comunes a todas las configuraciones
if(MSVC)
    add_compile_options(
        /W4                 # Nivel alto de advertencias
        /utf-8              # Codificación UTF-8
        /permissive-        # Conformidad estricta con el estándar
        /DNOMINMAX          # Evitar colisión de macros min/max en Windows
    )
else()
    add_compile_options(
        -Wall               # Advertencias base
        -Wextra             # Advertencias adicionales
        -Wpedantic          # Adherencia rigurosa al estándar C++
    )
endif()

# Con MinGW, enlazar estático: así los .exe no dependen de libstdc++-6.dll ni de
# libgcc_s_seh-1.dll y funcionan en cualquier PC con Windows, sin instalar MinGW.
if(MINGW)
    add_link_options(-static)
endif()

# Directorios de inclusión globales
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Librería núcleo (Core Library)
add_library(geodrone_nucleo STATIC
    src/geometria/Punto.hpp
    src/geometria/Segmento.hpp
    src/geometria/Poligono.hpp
    src/geometria/Geometria.hpp
    src/geometria/Geometria.cpp
    src/geometria/EnvolventeConvexa.hpp
    src/grafo/GrafoVisibilidad.hpp
    src/grafo/Dijkstra.hpp
    src/grafo/GrafoMision.hpp
    src/grafo/GrafoMision.cpp
    src/grafo/ArbolMinimo.hpp
    src/grafo/RedRecarga.hpp
    src/grafo/RedRecarga.cpp
    src/escenario/Escenario.hpp
    src/escenario/Aleatorio.hpp
    src/escenario/ArchivoEscenario.hpp
    src/escenario/ArchivoEscenario.cpp
    src/escenario/GeneradorDatasets.hpp
    src/escenario/GeneradorDatasets.cpp
    src/solvers/SolverExacto.hpp
    src/solvers/SolverExacto.cpp
    src/solvers/SolverHeuristico.hpp
    src/solvers/SolverHeuristico.cpp
    src/solvers/PlanificadorRecargas.hpp
    src/solvers/PlanificadorRecargas.cpp
    src/solvers/Ruta.hpp
    src/solvers/SolverMision.hpp
    src/solvers/SolverMision.cpp
    src/juez/UsoMemoria.hpp
    src/juez/UsoMemoria.cpp
    src/juez/Juez.hpp
    src/juez/Juez.cpp
    src/exportacion/Exportador.hpp
    src/exportacion/Exportador.cpp
)

target_include_directories(geodrone_nucleo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Memoria pico del proceso: GetProcessMemoryInfo está en psapi en Windows.
if(WIN32)
    target_link_libraries(geodrone_nucleo PUBLIC psapi)
endif()

# 1. Ejecutable Principal / CLI
add_executable(geodrone
    src/main.cpp
)
target_link_libraries(geodrone PRIVATE geodrone_nucleo)

# 2. Ejecutable de Pruebas Unitarias de Geometría
add_executable(prueba_geometria
    tests/prueba_geometria.cpp
)
target_link_libraries(prueba_geometria PRIVATE geodrone_nucleo)

# 3. Pruebas de lectura y escritura de escenarios
add_executable(prueba_escenario
    tests/prueba_escenario.cpp
)
target_link_libraries(prueba_escenario PRIVATE geodrone_nucleo)

# 4. Pruebas del generador de datasets
add_executable(prueba_generador
    tests/prueba_generador.cpp
)
target_link_libraries(prueba_generador PRIVATE geodrone_nucleo)

# 5. Pruebas del grafo de misión (visibilidad, Dijkstra, caminos y DFS)
add_executable(prueba_grafo
    tests/prueba_grafo.cpp
)
target_link_libraries(prueba_grafo PRIVATE geodrone_nucleo)

# 6. Pruebas de la red de recarga (Prim, Tarjan, búsqueda binaria e inserción de recargas)
add_executable(prueba_red
    tests/prueba_red.cpp
)
target_link_libraries(prueba_red PRIVATE geodrone_nucleo)

# 7. Pruebas de los solvers (exacto contra fuerza bruta)
add_executable(prueba_solvers
    tests/prueba_solvers.cpp
)
target_link_libraries(prueba_solvers PRIVATE geodrone_nucleo)

# 8. Pruebas de la misión completa (resolver, recargas, trayectoria y exportación)
add_executable(prueba_mision
    tests/prueba_mision.cpp
)
target_link_libraries(prueba_mision PRIVATE geodrone_nucleo)

# 9. Pruebas del juez (verificador, veredictos, ranking y benchmark)
add_executable(prueba_juez
    tests/prueba_juez.cpp
)
target_link_libraries(prueba_juez PRIVATE geodrone_nucleo)

# Habilitar suite de tests con CTest
enable_testing()
add_test(NAME PruebasGeometria COMMAND prueba_geometria)
add_test(NAME PruebasEscenario COMMAND prueba_escenario ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
add_test(NAME PruebasGenerador COMMAND prueba_generador)
add_test(NAME PruebasGrafo COMMAND prueba_grafo ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
add_test(NAME PruebasRed COMMAND prueba_red ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
add_test(NAME PruebasSolvers COMMAND prueba_solvers ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
add_test(NAME PruebasMision COMMAND prueba_mision ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
add_test(NAME PruebasJuez COMMAND prueba_juez ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
