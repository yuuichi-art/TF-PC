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

# Opciones de compilación optimizada según compilador
if(MSVC)
    add_compile_options(
        /O2                 # Máxima optimización de velocidad
        /W4                 # Nivel alto de advertencias
        /utf-8              # Codificación UTF-8
        /permissive-        # Conformidad estricta con el estándar
        /DNOMINMAX          # Evitar colisión de macros min/max en Windows
    )
else()
    add_compile_options(
        -O3                 # Nivel de optimización 3
        -Wall               # Advertencias base
        -Wextra             # Advertencias adicionales
        -Wpedantic          # Adherencia rigurosa al estándar C++
    )
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
    src/graph/VisibilityGraph.hpp
    src/graph/Dijkstra.hpp
    src/solvers/ExactSolver.hpp
    src/solvers/HeuristicSolver.hpp
    src/judge/DatasetGenerator.hpp
)

target_include_directories(geodrone_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

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

# Habilitar suite de tests con CTest
enable_testing()
add_test(NAME GeometryUnitTests COMMAND test_geometry)
