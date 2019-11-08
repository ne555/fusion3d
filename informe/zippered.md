Mesh zippering
===

1. Eliminar área solapada
2. Clipear (¿?) una superficie contra la otra
3. Eliminar triángulos pequeños

# Eliminar área solapada
repetir:
	* Recorre el perímetro de A
	* si $T_A \in B$ elimina T_A
	* lo mismo para B
hasta que no haya cambios en A y B

# Clipeo (A contra B)
Agrega vértices en la intersección de los segmentos de A contra los de B
Elimina los vértices de A que caen dentro de B
Retriangular

## Pasaje a 3D
los segmentos en el perímetro de B se proyectan según la normal de la cara
entonces se intersecta los segmentos de A contra los planos normales en dirección de los segmentos de B

# Elimina triángulos pequeños
¿hace falta?
si la altitud (¿?) está por debajo de un umbral
preferir matar a los nuevos producto del clipeo

# Falsa arista
puntos cerca de la silueta, en esquinas (bordes)
se necesita una captura de ambos lados del borde
no se sabe qué hacer

Consenso
===

luego de zip se tiene la _topología_
para detalle, se necesita refinar 

reintroducir la información descartada:
	* ajustar los vértices de la zippered a un promedio ponderado de las posiciones originales
	* se mueve solamente según la normal de la superficie

1. Aproximación local a la normal:
	promedio de las normales de los vértices dentro de una esfera centrada en V
2. Intersectar esta normal contra cada superficie
3. Promedio ponderado de esas intersecciones

