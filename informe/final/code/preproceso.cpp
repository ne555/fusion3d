auto nube = load_cloud_ply("bun000.ply");
// suavizado del ruido
double resolucion = cloud_resolution(nube);
auto nube_sin_ruido = moving_least_squares(nube,
                                           4 * resolution);
// eliminación de puntos aislados, puntos de borde
// y puntos con normales alejadas a la cámara
auto nube_limpia = preprocess(nube_sin_ruido);
