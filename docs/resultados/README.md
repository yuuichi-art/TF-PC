# Resultados del benchmark final

Evidencia para la sección "Pruebas y resultados" del informe. Los datos completos están en [`results.csv`](results.csv) (una fila por escenario y algoritmo) y [`ranking.csv`](ranking.csv).

## Entorno

- Intel Core i5-10400F a 2,90 GHz, 16 GB de RAM, Windows 10.
- g++ 16.2.0 (MSYS2, UCRT64), CMake 4.4, compilación Release (`-O3 -DNDEBUG`) enlazada estática.
- Los tiempos dependen del equipo; los costos no, salvo en el caso que se explica en el hallazgo 5.

## Cómo reproducirlo

1. Compilar en Release y abrir `./build/geodrone` desde la raíz del proyecto.
2. Opción 1 con los valores por defecto: genera `data/generated/` (5 semillas por categoría más `extreme_max`). Salir.
3. Volver a abrir el programa y elegir **primero** la opción 3 con los valores por defecto. El juez debe ser lo primero que corre, porque la memoria pico del proceso nunca baja; si antes se generaron datasets, la columna de memoria queda tapada por ese pico, y el programa lo avisa.

## Conjunto evaluado

| Grupo | Casos | Clientes | Algoritmos por caso | Ejecuciones |
| --- | --- | --- | --- | --- |
| Small | 5 | 3–10 | 6 (incluye el exacto) | 30 |
| Medium | 5 | 19–33 | 5 | 25 |
| Large | 5 | 49–149 | 5 | 25 |
| Extreme | 5 + `extreme_max` | 400–629 y 1000 | 5 | 30 |
| Adversarial | 5 | 3–5 | 6 (incluye el exacto) | 30 |
| **Total** | **26** | | | **140** |

`extreme_max` es el peor caso permitido: todos los rangos de Extreme en su máximo (1000 clientes, 25 estaciones, 150 obstáculos).

## Veredictos

**140 OK, 0 WA, 0 TLE, 0 MLE.** Todas las rutas pasaron la verificación independiente del juez: sin cruzar obstáculos, con batería suficiente en cada tramo, visitando a cada cliente una vez o excluyéndolo con motivo real, y con costo y recargas coherentes.

## Tiempo y memoria

Peor caso de cada categoría. El tiempo es preparación (grafo de misión y red de recarga) más resolución.

| Categoría | Peor tiempo | Límite | Memoria pico | Límite |
| --- | --- | --- | --- | --- |
| Small | 3,4 ms | 1 s | 4,7 MB | 256 MB |
| Medium | 3,4 ms | 2 s | 4,9 MB | 256 MB |
| Large | 84 ms | 3 s | 6,7 MB | 512 MB |
| Extreme (semillas 1–5) | 1,13 s | 5 s | — | 512 MB |
| Extreme (`extreme_max`) | 4,50 s | 5 s | 33,2 MB | 512 MB |
| Adversarial | 0,2 ms | 1 s | 4,7 MB | 256 MB |

En `extreme_max`, la preparación tarda 2,26 s (45 % del límite). Las técnicas constructivas terminan en unos 2,28 s, y la búsqueda iterada usa el resto del presupuesto hasta el 90 % del límite.

## Ranking por categoría

Gap medio contra la mejor ruta encontrada en cada caso (con recargas). Orden: más casos OK, luego menor gap, luego menor tiempo.

| Posición | Small | Medium | Large | Extreme |
| --- | --- | --- | --- | --- |
| 1 | búsqueda iterada 0,99 % | búsqueda iterada 0,95 % | búsqueda iterada 0,71 % | búsqueda iterada 0,43 % |
| 2 | branch and bound 0,99 % | 2-aprox. + 2-opt 1,36 % | 2-aprox. + 2-opt 3,62 % | vecino + 2-opt 3,79 % |
| 3 | vecino + 2-opt 4,46 % | vecino + 2-opt 5,41 % | vecino + 2-opt 4,36 % | 2-aprox. + 2-opt 4,51 % |
| 4 | 2-aprox. + 2-opt 8,31 % | vecino 17,56 % | vecino 14,78 % | vecino 15,14 % |
| 5 | vecino 8,93 % | 2-aproximación 24,51 % | 2-aproximación 25,14 % | 2-aproximación 27,39 % |
| 6 | 2-aproximación 16,28 % | — | — | — |

Tiempo medio de la búsqueda iterada: 0,35 ms en Small, 2,5 ms en Medium, 41 ms en Large y 1,41 s en Extreme (contando `extreme_max`).

En los adversariales todas las técnicas empatan con gap 0 (tienen de 3 a 5 clientes), así que su orden en el ranking solo refleja diferencias de tiempo de microsegundos.

## Hallazgos

1. **La búsqueda local iterada gana en todas las categorías**, con un gap medio menor a 1 %, y en Extreme cuesta más tiempo que el resto, aunque dentro del límite.
2. **2-opt aporta la mayor parte de la mejora.** El vecino más cercano queda entre 8,9 % y 17,6 % sobre la mejor ruta; con 2-opt baja a entre 3,8 % y 5,4 %.
3. **La 2-aproximación es la peor en la práctica (16 % a 27 %), pero es la única con garantía** (≤ 2 × óptimo). Mejorada con 2-opt es competitiva: segunda en Medium y Large.
4. **El exacto optimiza el orden, no la ruta con recargas (decisión 5).** En `small_3`, su orden óptimo da una ruta de 4583,81 y la 2-aproximación + 2-opt llega a 4367,91 (4,9 % menos), porque su orden necesita desvíos más cortos para recargar. Las rutas están en [`rutas/`](rutas/).
5. **En `extreme_max` actúa el resguardo de tiempo.** Tras 2,26 s de preparación, a la búsqueda iterada no le alcanza el presupuesto para sus 200 iteraciones y se detiene al 90 % del límite (4,50 s de 5). El veredicto es OK, pero su costo puede variar levemente entre corridas, por ejemplo entre 33 918 y 33 953, según dónde se corte. Es el único caso no reproducible al 100 %. Además, una ruta con menos iteraciones puede salir más corta: el costo del orden siempre baja con cada iteración, pero el de la ruta con recargas no necesariamente (el mismo efecto del hallazgo 4). En un equipo unas 2,2 veces más lento que el de la medición, la preparación de `extreme_max` por sí sola llegaría al límite de 5 s.
6. **La cota del árbol de expansión mínima garantiza poco.** El `gap_cota` de la búsqueda iterada va de 61 % a 85 %: el árbol ignora los desvíos para recargar y un árbol cuesta bastante menos que un ciclo. Sirve como garantía (ninguna ruta puede estar más lejos del óptimo), pero para comparar técnicas se usa `gap_mejor`.
7. **Robustez ante casos adversariales.** Las 30 rutas son válidas y cada caso se comporta como se esperaba:
   - `adv_enclosed`: excluye a C1 por obstáculos;
   - `adv_unreachable_battery`: excluye a C4 por batería, con 1 recarga;
   - `adv_corridor`: cruza el pasillo de ancho 2 en línea recta;
   - `adv_concave_trap`: rodea la U;
   - `adv_collinear`: vuela sobre los bordes, que está permitido.
8. **Las recargas crecen con el tamaño.** La búsqueda iterada hace en promedio 1,8 recargas en Small, 5,2 en Medium, 11,2 en Large y 35,2 en Extreme, con la batería fijada en 1,2 × la autonomía mínima.

## Rutas de ejemplo (`rutas/`)

| Archivo | Qué muestra |
| --- | --- |
| `adv_concave_trap_branch_and_bound.txt` | Rodeo de la trampa cóncava por los vértices de la U. |
| `adv_enclosed_branch_and_bound.txt` | Cliente encerrado excluido por obstáculos. |
| `adv_unreachable_battery_branch_and_bound.txt` | Cliente excluido por batería y una recarga en la estación. |
| `small_3_branch_and_bound.txt` y `small_3_mst_approximation_2opt.txt` | El hallazgo 4: orden óptimo frente a ruta final más corta. |
| `medium_1_iterated_local_search.txt` | Ruta con varias recargas en estaciones y en la base. |
