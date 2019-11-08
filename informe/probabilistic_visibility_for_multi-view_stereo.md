Probabilistic fusion
===

Para distinguir fondo/figura se usa ballooning term.
hace desaparecer detaller y estructuras finas.
propuesta: intelligent ballooning term.

Función de costo en el volumen, se extrae una superficie.
Explotar consistencia fotográfica: ¿qué tan consistente es la reconstrucción?
se puede distinguir si el punto está en la superficie pero no si está dentro o fuera -> se usa el ballooning

propaga el valor de consistencia hacia la cámara (¿carving?) determinando ese trayecto como fondo (no hay oclusión)

## Graph-cuts
Función de costo, términos:
	* Labeling: fondo o figura
	* Discontinuidad: estar en el límite

E = \int_S \rho(x) dA + \int_V \sigma(x) dV
S es el límite entre fonto y figura
V es el volumen de la figura
\sigma(x) balloning term
