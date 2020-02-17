Conclusiones
===

#Del producto
	En este proyecto se realizó el desarrollo de una biblioteca de software para
	realizar la reconstrucción tridimensional de un objeto a partir de capturas de
	vistas parciales.
	Para esto, se dividió el problema en tres módulos: registración, fusión y
	rellenado de huecos, y se implementaron diversos algoritmos.

	Al trabajar directamente con las nubes de puntos no se restringió el
	dispositivo de captura a un hardware en particular. Sin embargo, las
	restricciones impuestas de base giratoria y limitar la cantidad de capturas
	requeridas fueron planteadas en base al trabajo realizado por %\cite{Pancho}.

	Las registraciones se obtuvieron en tiempos razonables, sin requerir hardware especial.
	Se logró contener la propagación de los errores de registración.

	El hecho de usar una base giratoria genera demasiadas oclusiones, los huecos
	son de tamaño considerable. Aún así, el resultado es una malla cerrada, y la
	superficie estimada en las zonas sin información se une suavemente al resto de
	la malla.

#Del proyecto
La elección de la metodología en cascada modificada fue incorrecta.
La investigación bibliográfica se alargó demasiado y se desperdiciaron recursos
al abordar temas que tuvieron que ser descartados luego (como el uso de la
información de textura).
Además, se produjo un desfasaje entre la adquisición de los conocimientos y la puesta a prueba de los mismos.

Hubiese sido mejor utilizar directamente una metodología incremental en todo el
proceso y agregar un primer módulo de preproceso que contenga
la reducción de ruido y la operatoria básica con las nubes de puntos.


@@
##Base de datos (mejor en la sección de materiales y métodos)
Una vez que se determinó que no se contaría con una base de datos propia,
se realizó la búsqueda de un repositorio adecuado.
_redwood_
_freibug_
	rgb y profundidad, pero el movimiento es pequeño y libre
	(tendría que eliminar intermedios)
_middlebury_
	base giratoria, pero sólo RGB
	(tendría que generar el mapa de profundidad)
_stanford_
	base giratoria, nube de puntos, sin textura.
	Se optó por esta.
	Se decidió no generar artificialmente los puntos de textura para tener un
	caso más real.

@@Impresión 3D: ¿con watertight es suficiente?

@@¿también en materiales y métodos?
##Tecnología: PCL
@@instalación
@@¿pongo un apartado de riesgos efectivizados?
Primeramente, se contaba con un sistema operativo Arch Linux, donde la
instalación de esta biblioteca se realiza mediante un script PKGBUILD que
resuelve las dependencias y configura los módulos.
Fue necesario compilar los fuentes, pero fuera del tiempo requerido, no se
tuvieron mayores inconvenientes.

Cuando, por razones externas al proyecto, fue necesario reemplazar el equipo,
se contaba esta vez con un sistema Clear Linux.
Ahora la instalación resultó más problemática.
Se requerían demasiados recursos de memoria, por lo que el sistema operativo
detenía el proceso.
Fue necesario un largo proceso de configuración y prueba para lograr la instalación exitosa de la biblioteca.


@@uso (materiales y métodos)
Debido al uso intensivo de código templatizado en PCL, la compilación del
código cliente requería de un tiempo considerable (aproximadamente un minuto).


##Riesgos efectivizados
Ausencia de repositorio de mallas tridimensionales
(copiar base de datos)

Falla en los equipos de trabajo
(copiar parte de pcl)
Meshlab: no se logró instalarlo en el nuevo equipo, se cambió a CloudCompare



Trabajos futuros
===
Combinar reconstrucciones del objeto en varias posiciones sobre la base
giratoria con el fin de eliminar huecos y ajustar el error de inflación de la
registración.

Ajustar los métodos para trabajar con el volumen de puntos generados por %\cite{Pancho}.
Paralelizar.

Mejorar la detección de outliers en el módulo de fusión.

Modificar el método de advancing front para que utilice una superficie de soporte para generar los nuevos puntos.
