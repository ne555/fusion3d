def iss_keypoint(nube):
  keypoints = []
  for p in nube.puntos:
    vecinos = obtener_puntos_cercanos(nube, p, radio)
    m = matriz_de_covarianza(vecinos)
    %$\lambda$% = eigenvalues(m)
    if %$\lambda_1/\lambda_2 > \mbox{umbral}_1$% and %$\lambda_2/\lambda_3 > \mbox{umbral}_2$%:
        keypoints.add(p)
  return keypoints
