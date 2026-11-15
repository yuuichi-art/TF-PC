cmake_minimum_required(VERSION 3.16)

project(GeoDroneChallenge
    VERSION 1.0.0
    DESCRIPTION "Optimizacion de Rutas de Entrega con Drones en Entornos Urbanos - USIL 2026"
    LANGUAGES CXX
)

# Estándar de C++: C++17 o C++20
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
add_library(geodrone_core STATIC
    src/geometry/Point.hpp
    src/geometry/Segment.hpp
    src/geometry/Polygon.hpp
    src/geometry/GeometryUtils.hpp
    src/geometry/GeometryUtils.cpp
    src/geometry/ConvexHull.hpp
    src/graph/VisibilityGraph.hpp
    src/graph/Dijkstra.hpp
    src/graph/MissionGraph.hpp
    src/graph/MissionGraph.cpp
    src/graph/Mst.hpp
    src/graph/ChargingNetwork.hpp
    src/graph/ChargingNetwork.cpp
    src/scenario/Scenario.hpp
    src/scenario/ScenarioIO.hpp
    src/scenario/ScenarioIO.cpp
    src/scenario/DatasetGenerator.hpp
    src/scenario/DatasetGenerator.cpp
    src/solvers/ExactSolver.hpp
    src/solvers/ExactSolver.cpp
    src/solvers/HeuristicSolver.hpp
    src/solvers/HeuristicSolver.cpp
    src/solvers/RechargePlanner.hpp
    src/solvers/RechargePlanner.cpp
    src/solvers/Route.hpp
    src/solvers/MissionSolver.hpp
    src/solvers/MissionSolver.cpp
    src/judge/MemoryUsage.hpp
    src/judge/MemoryUsage.cpp
    src/report/Exporter.hpp
    src/report/Exporter.cpp
)

target_include_directories(geodrone_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Memoria pico del proceso: GetProcessMemoryInfo está en psapi en Windows.
if(WIN32)
    target_link_libraries(geodrone_core PUBLIC psapi)
endif()

# 1. Ejecutable Principal / CLI
add_executable(geodrone
    src/main.cpp
)
target_link_libraries(geodrone PRIVATE geodrone_core)

# 2. Ejecutable de Pruebas Unitarias de Geometría
add_executable(test_geometry
    tests/test_geometry.cpp
)
target_link_libraries(test_geometry PRIVATE geodrone_core)

# 3. Pruebas de lectura y escritura de escenarios
add_executable(test_scenario
    tests/test_scenario.cpp
)
target_link_libraries(test_scenario PRIVATE geodrone_core)

# 4. Pruebas del generador de datasets
add_executable(test_generator
    tests/test_generator.cpp
)
target_link_libraries(test_generator PRIVATE geodrone_core)

# 5. Pruebas del grafo de misión (visibilidad, Dijkstra, caminos y DFS)
add_executable(test_graph
    tests/test_graph.cpp
)
target_link_libraries(test_graph PRIVATE geodrone_core)

# 6. Pruebas de la red de recarga (Prim, Tarjan, búsqueda binaria e inserción de recargas)
add_executable(test_network
    tests/test_network.cpp
)
target_link_libraries(test_network PRIVATE geodrone_core)

# 7. Pruebas de los solvers (exacto contra fuerza bruta)
add_executable(test_solvers
    tests/test_solvers.cpp
)
target_link_libraries(test_solvers PRIVATE geodrone_core)

# 8. Pruebas de la misión completa (resolver, recargas, trayectoria y exportación)
add_executable(test_mission
    tests/test_mission.cpp
)
target_link_libraries(test_mission PRIVATE geodrone_core)

# Habilitar suite de tests con CTest
enable_testing()
add_test(NAME GeometryUnitTests COMMAND test_geometry)
add_test(NAME ScenarioTests COMMAND test_scenario ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
add_test(NAME GeneratorTests COMMAND test_generator)
add_test(NAME GraphTests COMMAND test_graph ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
add_test(NAME NetworkTests COMMAND test_network ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
add_test(NAME SolverTests COMMAND test_solvers ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
add_test(NAME MissionTests COMMAND test_mission ${CMAKE_CURRENT_SOURCE_DIR}/data/adversarial)
