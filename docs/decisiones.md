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
| RF05 Grafo de navegación | Grafo de visibilidad por pares, con filtro por caja envolvente | Implementado |
| RF06 Caminos mínimos | Dijkstra desde cada nodo clave, sin expandir otros nodos clave | Implementado |
| RF07 Exacto (Small) | Branch and bound con cota de árbol de expansión mínima | Implementado |
| RF08 Heurísticas | Vecino más cercano, 2-aproximación por árbol de expansión mínima, 2-opt, búsqueda local iterada | Implementado |
| RF09 Autonomía | Los solvers optimizan el orden; un paso común inserta las recargas (ver sección 4) | Implementado |
| RF10 Búsqueda binaria | Menor autonomía con la que la red de recarga permite atender a todos los clientes (ver sección 4) | Implementado |
| RF11 Análisis estructural | DFS: componentes conexas (clientes inalcanzables) y puntos de articulación (Tarjan) en la red de recarga | Implementado |
| RF12 Red mínima | Prim entre la base y las estaciones | Implementado |
| RF13 Datasets | Generador con semilla, casos aleatorios y adversariales | Implementado |
| RF14 Juez | Verificación de rutas, tiempo, memoria y gap | Implementado |
| RF15 Exportación | TXT y CSV; SVG al final | Implementado |

La red de recarga tiene como nodos la base y las estaciones. Dos nodos se conectan si su distancia de vuelo no supera la autonomía.

El solver exacto admite hasta 12 clientes. Medición (Release `-O3`, peor de 10 instancias por fila; el árbol completo con 10 clientes tiene más de 3,6 millones de hojas):

| Instancia | Clientes | Peor tiempo | Peores nodos visitados |
| --- | --- | --- | --- |
| Small generados con obstáculos (50 escenarios) | 10 | 24 ms | 35 997 |
| Puntos uniformes | 10 / 12 | 14 ms / 49 ms | 21 159 / 54 231 |
| Puntos en racimos | 10 / 12 | 12 ms / 75 ms | 14 742 / 99 401 |
| Puntos en círculo o cuadrícula | 12 | menos de 1 ms | menos de 1 000 |

Heurísticas sobre el orden de visita (Release `-O3`, semillas 1 a 5 de cada categoría, sin recargas). El gap contra la mejor ruta encontrada compara las técnicas entre sí; el gap contra la cota es un máximo garantizado, porque el árbol de expansión mínima nunca cuesta más que el óptimo:

| Categoría | Vecino más cercano | 2-aproximación | Vecino + 2-opt | 2-aproximación + 2-opt | Búsqueda local iterada (200) |
| --- | --- | --- | --- | --- | --- |
| Small: gap contra el óptimo | 6,6 % | 14,5 % | 1,2 % | 1,5 % | 0 % |
| Medium: gap contra la mejor | 14,4 % | 29,3 % | 3,8 % | 1,0 % | 0 % |
| Large: gap contra la mejor | 19,9 % | 34,1 % | 7,3 % | 6,3 % | 0 % |
| Extreme: gap contra la mejor | 19,3 % | 32,4 % | 4,2 % | 6,3 % | 0 % |
| Extreme: peor tiempo | 0,6 ms | 1,9 ms | 3,7 ms | 4,1 ms | 589 ms |

La búsqueda local iterada usa 200 iteraciones fijas, para que cada semilla dé siempre la misma ruta, y un límite de tiempo como resguardo. El resguardo solo actúa en el peor caso `extreme_max`, cuyo costo puede variar levemente entre corridas (ver `docs/resultados/`). Más iteraciones mejoran poco y no caben en el peor caso Extreme (gap contra la cota, promedio de 3 semillas):

| Iteraciones | 0 (solo 2-opt) | 50 | 200 | 500 | 1000 |
| --- | --- | --- | --- | --- | --- |
| Large | 27,3 % | 18,6 % | 17,7 % | 17,3 % | 16,6 % |
| Extreme | 21,6 % | 18,2 % | 16,3 % | 15,5 % | 14,9 % |
| Extreme: peor tiempo | 4 ms | 0,14 s | 0,58 s | 1,35 s | 2,90 s |

## 4. Recargas y búsqueda binaria

`dRecarga(c)` es la distancia de `c` al punto de recarga más cercano entre los alcanzables desde la base (la base o una estación).

Inserción de recargas, recorriendo el orden que entrega el solver:

1. Si con la batería actual se llega al siguiente destino y al llegar queda al menos `dRecarga(destino)`, se vuela directo.
2. Si no, se busca el desvío más corto por la red de recarga: un Dijkstra sobre los puntos de recarga, que parte de los alcanzables con la batería actual y termina en uno desde el cual el destino es seguro con la batería llena. Desviarse a una sola estación es el caso de un salto; si hace falta, el camino encadena varias.
3. Si no existe ese camino, la ruta se marca como infactible.

Búsqueda binaria de la autonomía mínima `B`:

- `B` es factible si, en la red de recarga con aristas de longitud ≤ `B`, cada cliente `c` tiene un punto de recarga alcanzable `s` con `d(s, c) + dRecarga(c) ≤ B`.
- Como el mejor `s` es la recarga más cercana, la condición se reduce a `2 · dRecarga(c) ≤ B`: llegar desde una recarga y volver a una recarga.
- La condición es monótona y coincide con la condición bajo la cual la inserción de recargas siempre encuentra ruta.
- Se busca sobre reales, entre 0 y el doble de la mayor distancia de la matriz, con 60 iteraciones fijas. Hace falta el doble porque un cliente lejano puede exigir ir y volver.
- La red compara de forma exacta. La tolerancia de 1e-6 solo se usa al simular la batería tramo a tramo, en el planificador y en el juez; así la autonomía mínima siempre deja ese margen a la simulación.

Los escenarios generados usan como batería la autonomía mínima × 1,2, redondeada hacia arriba: todos los clientes se pueden atender y aun así hacen falta recargas.

## 5. Programa y juez

- Un solo programa de consola, con un módulo para resolver escenarios y otro para el juez.
- Se maneja con un menú interactivo: generar datasets, resolver un escenario, ejecutar el juez y analizar la red de recarga.
- Se ejecuta desde la raíz del proyecto: las rutas por defecto (`data/generated/`, `output/`) son relativas a ella.
- Con MinGW se enlaza estático, para que el `.exe` funcione en cualquier PC con Windows sin instalar MinGW.
- Código en español, sin tildes ni ñ: `PascalCase` para tipos, `camelCase` para funciones y variables, y `MAYUSCULAS` para constantes. Los comentarios también van en español.
  - **Términos en inglés:** se mantienen los que la guía usa en inglés (*solver*, *dataset*, *benchmark*, *ranking*, *gap*, *branch and bound*, *2-opt*) y los nombres propios (Dijkstra, Prim, Tarjan).
  - **Formatos:** los de §6 no se traducen, porque son datos y no código.
- Resolver un escenario prepara una sola vez el grafo de misión y la red de recarga, y luego aplica el algoritmo elegido. El tiempo informado es preparación + resolución.
- La búsqueda local iterada se detiene antes de llegar al 90 % del tiempo límite de la categoría, descontando la preparación.
- Algoritmos, con el nombre que aparece en los archivos: `branch_and_bound` (solo hasta 12 clientes atendibles), `nearest_neighbor`, `mst_approximation`, `nearest_neighbor_2opt`, `mst_approximation_2opt` e `iterated_local_search` (parte del vecino más cercano y usa la semilla del escenario).
- El menú valida la entrada numérica, vacía la salida en cada mensaje (necesario con `cin.tie(NULL)`), lee rutas con `getline` y ofrece valores por defecto con Enter.
- La opción del juez ejecuta todos los casos de una vez.
- El juez verifica cada ruta con código independiente de los solvers: ningún tramo cruza un obstáculo, la batería nunca baja de 0, se visitan todos los clientes alcanzables y la ruta empieza y termina en la base.
- Veredictos: OK (ruta válida dentro del límite), WA (ruta inválida), TLE (excede el tiempo), MLE (excede la memoria).
- Ranking por categoría: más casos OK, luego menor gap medio, luego menor tiempo medio.
- El tiempo se mide con `<chrono>`. La memoria es el pico del proceso; el juez ejecuta los casos de menor a mayor para que la cifra corresponda al caso actual. Por eso el juez debe ser lo primero que se ejecuta al abrir el programa; si antes se usó otra opción, el menú avisa que la memoria no será por caso.

Detalles del juez:

- Menú: 1 generar datasets, 2 resolver un escenario, 3 ejecutar el juez, 4 analizar la red de recarga.
- Toma los `.txt` de una carpeta (por defecto `data/generated/`) y, si se pide, los de `data/adversarial/`.
- El exacto corre en Small y en los adversariales; las demás técnicas, en todos los casos.
- El grafo de misión se prepara una vez por caso; el tiempo de cada ruta es preparación + resolución.
- La verificación usa solo la ruta, el escenario y la geometría, y comprueba además que cada punto etiquetado está en las coordenadas del escenario y que el costo, las recargas y el orden informados coinciden con la trayectoria. Cada exclusión se confirma con el grafo de misión (DFS para `OBSTACLES`, 2·dRecarga > B para `BATTERY`).
- Precedencia de veredictos: WA si la ruta no es válida; si es válida, TLE si excede el tiempo, MLE si excede la memoria y OK en otro caso.
- `gap_mejor` es el porcentaje sobre la ruta OK más corta del caso. `gap_cota` es el porcentaje sobre el árbol de expansión mínima de la base y los clientes atendibles, que nunca supera al óptimo; es un máximo garantizado de cuánto se aleja la ruta del óptimo.
- Salidas: `output/judge/results.csv`, `output/judge/ranking.csv` y una ruta por caso y algoritmo en `output/judge/routes/`.

Consecuencia observada de optimizar primero el orden y después insertar las recargas (§3): el orden óptimo del exacto no siempre da la ruta más corta una vez insertadas las recargas. En `small_3`, el exacto queda en 4583,81 y la 2-aproximación + 2-opt en 4367,91: su orden, algo más largo, necesita desvíos más cortos para recargar.

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

Ruta exportada (TXT, en `output/<escenario>_<algoritmo>.txt`): resumen, clientes excluidos, orden de nodos clave y trayectoria completa.

- `STATUS` es `OK` o `INFEASIBLE`, y `COST` lleva 6 decimales.
- `EXCLUDED` da la cantidad y luego una línea por cliente con su motivo: `OBSTACLES` (los obstáculos lo separan de la base) o `BATTERY` (2·dRecarga > B).
- En `PATH`, cada punto lleva su etiqueta: `D0` base, `C3` cliente, `S1` estación, `V` vértice de obstáculo.
- Una ruta sin clientes es solo `D0`: el dron no despega.

```text
SCENARIO adv_enclosed
ALGORITHM branch_and_bound
STATUS OK
COST 293.416641
RECHARGES 0
EXCLUDED 1
C1 OBSTACLES
ORDER 4
D0 C3 C2 D0
PATH 5
10 10 D0
90 10 C3
90 90 C2
75 25 V
10 10 D0
```

Resultados del juez (CSV), una fila por escenario y algoritmo:

`escenario,categoria,clientes,algoritmo,veredicto,costo,recargas,excluidos,tiempo_ms,memoria_mb,gap_mejor,gap_cota`

Ranking del juez (CSV): `categoria,posicion,algoritmo,casos_ok,casos,gap_medio,tiempo_medio_ms`.

Dibujo de la ruta (SVG, junto al TXT al resolver un escenario desde el menú). Muestra obstáculos en gris, la trayectoria en azul, la base como cuadrado rojo, las estaciones como triángulos verdes, los clientes como puntos y los excluidos como cruces rojas, con título y leyenda. Con más de 40 clientes se omiten las etiquetas para que se lea. Es solo visualización: la validación oficial es la del juez.

## 7. Datasets y límites

| Categoría | Clientes | Estaciones | Obstáculos | Vértices de obstáculos | Tiempo | Memoria |
| --- | --- | --- | --- | --- | --- | --- |
| Small | 1–10 | 1–3 | 1–5 | ≤ 30 | 1 s | 256 MB |
| Medium | 11–40 | 3–6 | 5–15 | ≤ 100 | 2 s | 256 MB |
| Large | 41–200 | 6–12 | 15–50 | ≤ 400 | 3 s | 512 MB |
| Extreme | 201–1000 | 12–25 | 50–150 | ≤ 1500 | 5 s | 512 MB |

Los rangos tentativos se mantienen porque cumplen la regla acordada: construir el grafo de misión (grafo de visibilidad y matriz de distancias) usa como máximo la mitad del tiempo límite, incluso en el peor caso.

Medición del grafo de misión (Release `-O3`, equipo de desarrollo). El peor caso usa todos los rangos en su máximo:

| Caso | Nodos | Aristas | Grafo + matriz |
| --- | --- | --- | --- |
| Large, peor caso | 445 | 21 554 | 0,08 s |
| Extreme, semillas 1 y 2 | 871–1 005 | 86 096–108 135 | 0,38–0,55 s |
| Extreme, peor caso | 1 735 | 198 915 | 2,2 s |

Para que el peor caso Extreme cupiera (antes tardaba 3,4 s) se hicieron dos cambios, sin alterar los resultados:

- La validación de tramos ya no crea vectores en cada llamada; se llama millones de veces al construir el grafo.
- Dijkstra no expande la base, las estaciones ni los clientes, porque un camino mínimo entre obstáculos solo dobla en vértices de obstáculos. Da las mismas distancias en la mitad de tiempo (de 1,77 s a 0,85 s en el peor caso Extreme).

Los casos hechos a mano de `data/adversarial/` usan la categoría `ADVERSARIAL`, con los mismos límites que Small, y semilla 0.

El set de benchmark incluye además `extreme_max`: un Extreme con todos los rangos en su máximo (1000 clientes, 25 estaciones, 150 obstáculos), para comprobar que el peor caso permitido cabe en el límite. En el benchmark final tardó 4,50 s de 5 con 33 MB de memoria. Los resultados completos están en [`docs/resultados/`](resultados/README.md).

## 8. Fuera de alcance

- Varios drones.
- Demanda por cliente y capacidad de carga.
- Tasa de consumo distinta de la distancia.
- Prioridades, ventanas de tiempo y clientes opcionales.
- Held-Karp u otro exacto adicional.
- Un segundo ejecutable para el juez.

## 9. Por definir

Nada por ahora.
