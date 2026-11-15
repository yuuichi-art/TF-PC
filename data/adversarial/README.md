# Casos adversariales

Escenarios hechos a mano, en un área de 100×100, para probar situaciones difíciles que el generador aleatorio no produce. Usan la categoría `ADVERSARIAL` (mismos límites que Small) y semilla 0.

| Archivo | Qué pone a prueba | Comportamiento esperado |
| --- | --- | --- |
| `adv_collinear.txt` | Base, clientes y estación alineados con los bordes del obstáculo y entre sí. | Los vuelos que van sobre la línea de un borde son válidos; la línea vertical x = 50 cruza el obstáculo y obliga a rodearlo. |
| `adv_enclosed.txt` | Cliente (50, 50) encerrado por cuatro muros que se solapan, sin ningún hueco. | El cliente encerrado queda fuera de la componente de la base y se excluye y reporta. |
| `adv_corridor.txt` | Pasillo de ancho 2 entre dos obstáculos. | La ruta de la base a (90, 50) pasa por el pasillo en línea recta. |
| `adv_concave_trap.txt` | Obstáculo cóncavo en forma de U abierto hacia la base, con un cliente dentro del hueco y otro detrás. | Al cliente del hueco se entra por la abertura; al de atrás se llega rodeando la U. |
| `adv_unreachable_battery.txt` | Cliente (95, 95) lejos de toda recarga con autonomía 60. | El cliente se excluye por batería y se reporta; los otros tres se atienden. |

Los muros de `adv_enclosed.txt` se solapan a propósito: si solo se tocaran, el dron podría volar sobre el borde compartido, porque tocar un borde está permitido.
