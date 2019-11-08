brisk: binary robust invariant scalable keypoints
====

detección espacio-escala:
	* criterio de saliencia
	* keypoints en capas en octavas y entrecapas
	* ajuste de función cuadrática

descripción:
	* patrón de puntos en un círculo concéntrico del keypoint
	* el gradiente de intensidad determina la dirección
	* binario

# método

## Detección espacio-scala
basado en AGAST
usa puntaje FAST como medida de saliencia

n octavas c_i y n entreoctavas d_i (típicamente n=4)
submuestreo (2) de la capa original
d_0 comienza como submuestreo (1.5) de c_0

máscara 9-16 ¿?
	9 píxeles consecutivos (¿?) en círculo de 16 diferentes al central
	fast 9-16 a cada octava	
		non-maxima supression en espacio-escala
			máximo respecto a sus 8 vecinos en la capa
			máximo respecto a los 2 de las capas superior e inferior

	saliencia es una medida continua en espacio y escala

## Descripción keypoints
cadena binaria que tiene los resultados de las comparaciones de brillo
dirección característica, normalizar por orientación (invarianza a rotaciones)

## Patrón de muestreo y estimación de rotación
círculos concéntricos, parecido a DAISY
kernel gausiano

## macheo
distancia hamming
	cantidad de bits distintos
