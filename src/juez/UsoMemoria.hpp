#ifndef GEODRONE_USOMEMORIA_HPP
#define GEODRONE_USOMEMORIA_HPP

namespace geodrone {

/**
 * @brief Memoria pico del proceso en MB: el máximo de memoria física (working set) que usó
 * desde que empezó. En Windows se lee con GetProcessMemoryInfo; en otros sistemas devuelve 0.
 *
 * Es un máximo histórico que nunca baja: por eso el juez ejecuta los casos de menor a mayor,
 * y así la cifra de cada caso corresponde al caso más grande ejecutado hasta ese momento.
 */
double memoriaPicoMB();

} // namespace geodrone

#endif // GEODRONE_USOMEMORIA_HPP
