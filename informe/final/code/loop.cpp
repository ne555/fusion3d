// vector registered
// convertir las transformaciones relativas en tranformaciones absolutas
for (size_t K = 1; K < registered.size(); ++K) {
  auto &prev = registered[K - 1];
  auto &current = registered[K];
  current.set_transformation(prev.get_transformation() *
                             current.get_transformation());
}
// realizar la corrección
loop_correction(registered);
