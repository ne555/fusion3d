Actor: usuario %creo que no hay otro

CU: Alinear dos mallas
	* _usuario_ selecciona malla _fuente_ y malla _destino_
	* el _sistema_ posiciona la malla fuente en el sistema de referencia de la
	  malla destino, devolviendo una matriz de transformación
		## Alt: no se encuentra la cantidad suficiente de correspondencias
			* el _sistema_ informa al _usuario_ que la malla _fuente_ no pudo
			  ser alineada
	* fin

@@CU: Detectar bucle
	@@ no es caso de uso
	@@ el usuario provee de una lista de imágenes circular que cierra.


CU: Alinear bucle
	% la acumulación de errores podría dar una espiral en lugar de algo cerrado
	* _usuario_ provee de una lista de mallas {m1, m2, ..., mn} que
	  corresponden a una trayectoria cerrada (¿?) en las capturas
	* _sistema_ llama _Alinear dos mallas_ de a pares
	* _sistema_ informa al usuario de mallas rechazadas 
		## Alt: demasiados rechazos o el bucle no cierra
			* ¿?
	* _sistema_ corrige acumulación de errores
	* resultado: lista de mallas válidas y sus transformaciones

@@CU: Rechazo (¿es caso de uso?)
	¿puedo rechazar mallas?
	¿tengo la capacidad?
	sí, pero no lo puedo resolver
	eliminar la malla y seguir no me alcanzan los ángulos

CU: Relleno de huecos
	¿qué  es hueco?
		borde de un sólo vecino
	detectar contorno hueco
	
	%¿métodos para elegir?
		triangular puntos en el borde
		difundir bordes
	¿qué es rellenar?

CU: Ajuste de parámetros
	* el usuario debe poder configurar los parámetros de los algoritmos
		ver algoritmo


CU: Extracción de superficie
	Entrada: lista de mallas
	Salida: triangulación
		los puntos se ajustarán como promedio ponderado,
		con factor de confianza según distancia al centro
	otra opción: truncated signed distance (superficie implícita)
	(técnicas volumétricas)
