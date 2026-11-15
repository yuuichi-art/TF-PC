# Decisiones de diseño — GeoDrone Challenge

Registro de las decisiones acordadas por el grupo. Si una decisión cambia, se actualiza aquí.

## 1. Modelo de misión

- Un solo dron por misión.
- La ruta es un ciclo: base → clientes → base.
- Todos los clientes son obligatorios y se visitan una vez.
- El costo es la distancia total volada, incluidos los desvíos a recargar.
- La batería se mide en unidades de distancia y empieza llena.
- La recarga es instantánea y completa, en las estaciones y en la base.
- Un cliente inalcanzable se excluye de la misión y se reporta; la ruta sigue con los demás.

## 2. Geometría

- Coordenadas en `double` con tolerancia `EPS = 1e-9`.
- Los datasets usan coordenadas enteras en un rango acotado.
- Los obstáculos son polígonos simples, convexos o cóncavos.
- Volar tocando un borde o un vértice está permitido; atravesar el interior, no.
- El generador aleatorio crea solo polígonos convexos sin solape, usando envolvente convexa.
- Los obstáculos cóncavos aparecen solo en casos adversariales hechos a mano.

## 3. Algoritmos

| Requerimiento | Algoritmo | Estado |
| --- | --- | --- |
| RF03, RF04 Geometría y validación | Producto cruz, intersección de segmentos, punto en polígono | Implementado |
| RF05 Grafo de navegación | Grafo de visibilidad por pares, con filtro por caja envolvente | Implementado, falta el filtro |
| RF06 Caminos mínimos | Dijkstra desde cada nodo clave | Implementado |
| RF07 Exacto (Small) | Branch and bound con cota de árbol de expansión mínima | Pendiente |
| RF08 Heurísticas | Vecino más cercano, 2-aproximación por árbol de expansión mínima, 2-opt, búsqueda local iterada | Pendiente |
| RF09 Autonomía | Los solvers optimizan el orden; un paso común inserta las recargas (ver sección 4) | Pendiente |
| RF10 Búsqueda binaria | Menor autonomía con la que la red de recarga permite atender a todos los clientes (ver sección 4) | Pendiente |
| RF11 Análisis estructural | DFS: componentes conexas (clientes inalcanzables) y puntos de articulación en la red de recarga | Pendiente |
| RF12 Red mínima | Prim sobre la red de recarga | Pendiente |
| RF13 Datasets | Generador con semilla, casos aleatorios y adversariales | Pendiente |
| RF14 Juez | Verificación de rutas, tiempo, memoria y gap | Pendiente |
| RF15 Exportación | TXT y CSV; SVG al final | Pendiente |

La red de recarga tiene como nodos la base y las estaciones. Dos nodos se conectan si su distancia de vuelo no supera la autonomía.

## 4. Recargas y búsqueda binaria

`dRecarga(c)` es la distancia de `c` al punto de recarga más cercano entre los alcanzables desde la base (la base o una estación).

Inserción de recargas, recorriendo el orden que entrega el solver:

1. Si con la batería actual se llega al siguiente destino y al llegar queda al menos `dRecarga(destino)`, se vuela directo.
2. Si no, se desvía al punto de recarga alcanzable que minimiza `d(actual, s) + d(s, destino)` y deja el viaje seguro.
3. Si ningún punto de recarga basta, se encadenan estaciones con Dijkstra sobre la red de recarga.
4. Si tampoco hay camino, la ruta se marca como infactible.

Búsqueda binaria de la autonomía mínima `B`:

- `B` es factible si, en la red de recarga con aristas de longitud ≤ `B`, cada cliente `c` tiene un punto de recarga alcanzable `s` con `d(s, c) + dRecarga(c) ≤ B`.
- La condición es monótona y coincide con la condición bajo la cual la inserción de recargas siempre encuentra ruta.
- Se busca sobre reales, entre 0 y la mayor distancia de la matriz, con un número fijo de iteraciones.

## 5. Programa y juez

- Un solo programa de consola, con un módulo para resolver escenarios y otro para el juez.
- Se maneja con un menú interactivo: generar datasets, resolver un escenario, ejecutar el juez y analizar la red de recarga.
- El menú valida la entrada numérica, vacía la salida en cada mensaje (necesario con `cin.tie(NULL)`), lee rutas con `getline` y ofrece valores por defecto con Enter.
- La opción del juez ejecuta todos los casos de una vez.
- El juez verifica cada ruta con código independiente de los solvers: ningún tramo cruza un obstáculo, la batería nunca baja de 0, se visitan todos los clientes alcanzables y la ruta empieza y termina en la base.
- Veredictos: OK (ruta válida dentro del límite), WA (ruta inválida), TLE (excede el tiempo), MLE (excede la memoria).
- Ranking por categoría: más casos OK, luego menor gap medio, luego menor tiempo medio.
- El tiempo se mide con `<chrono>`. La memoria es el pico del proceso; el juez ejecuta los casos de menor a mayor para que la cifra corresponda al caso actual.

## 6. Formatos de archivo

Escenario (TXT, etiquetas en orden fijo; cada obstáculo es su número de vértices seguido de sus coordenadas):

```text
NAME small_001
CATEGORY SMALL
SEED 42
AREA 1000 1000
BATTERY 450
DEPOT 500 500
STATIONS 2
200 300
800 700
CLIENTS 3
120 880
640 150
900 420
OBSTACLES 2
4  300 400  400 400  400 500  300 500
3  600 600  700 600  650 690
```

Ruta exportada (TXT): resumen, orden de nodos clave y trayectoria completa. En `PATH`, cada punto lleva su etiqueta: `D` base, `C` cliente, `S` estación, `V` vértice de obstáculo.

```text
SCENARIO small_001
ALGORITHM branch_and_bound
STATUS OK
COST 3521.47
RECHARGES 1
EXCLUDED 0
ORDER 6
D0 C2 C1 S1 C3 D0
PATH 9
500 500 D0
400 500 V
...
```

Resultados del juez (CSV), una fila por escenario y algoritmo:

`escenario,categoria,clientes,algoritmo,veredicto,costo,recargas,excluidos,tiempo_ms,memoria_mb,gap_mejor,gap_cota`

## 7. Datasets y límites

La tabla de categorías del README es tentativa. Los tamaños y límites finales se fijan después de medir tiempos reales.

Los casos hechos a mano de `data/adversarial/` usan la categoría `ADVERSARIAL`, con los mismos límites que Small, y semilla 0.

## 8. Fuera de alcance

- Varios drones.
- Demanda por cliente y capacidad de carga.
- Tasa de consumo distinta de la distancia.
- Prioridades, ventanas de tiempo y clientes opcionales.
- Held-Karp u otro exacto adicional.
- Un segundo ejecutable para el juez.

## 9. Por definir

- Tamaños y límites finales de cada categoría (se fijan tras medir).
