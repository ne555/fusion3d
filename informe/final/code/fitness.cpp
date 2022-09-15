for (size_t K = 1; K < registered.size(); ++K) {
    auto &prev = registered[K - 1];
    auto &current = registered[K];
    current.apply_transformation();
    auto [error_medio, desvio, solapamiento] = fitness(
        current.get_cloud(),
        prev.get_cloud(),
        4 * resolucion);
    //...
}
