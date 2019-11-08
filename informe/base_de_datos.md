Base de datos
===

## The Stanford 3D Scanning Repository

* armadillo (s|r)
* bunny     (s|r)
* dragon    (s|b|r)
* drill     (s|r)
* happy     (s|r|b)
* lucy      (unknown format .sd, transformaciones en .xf no concuerdan (ver entrada), .ply reconstrucción)
* rgb       (¿?) (no se usan)

información de posición de las capturas (transformaciones
[translación|rotación] para llevar todo a un mismo sistema de referencia)

algunas capturas tienen incorporado un backdrop (fondo)

en algunos casos se tienen las reconstrucciones con el método zippered
y método volumétrico

## Materiales y métodos
Escaneado con cyberware 3030 ms scanner
Lucy: Stanford large statue scanner (diseñado para el Digital Michelangelo Project)

triangulación (para obtener los puntos de la nube) calculada en hardware
Happy buddha y Dragoncon Brian Curless's spacetime analysis

las transformaciones se obtuvieron por alineación con ICP modificado (*ver*)
[tx, ty, tz, q]

combinación utilizando zippering o fusión volumétrica, ambos desarrollados por stanford (ZipPack, VripPack)
relleno de huecos volfill

otros modelos:
XYZ RGB cámaras auto-sincronizadas
resolución de 100 micrones = 0.1 mm

### Formato de archivo
archivos PLY

## Advertencia
las superficies son suaves
se eliminaron outliers, poco ruido, esconden malas alineaciones (¿?)
esconden muchos errores que suelen ocurrir en el escaneo 3D

_Los modelos no son una entrada realista para probar algoritmos de reconstrucción_

utilizar raw data

la conectividad es parte de la información del escáner Cyberware
se recomienda no descartarla

## Características de los modelos
### Stanford Bunny
Source: Stanford University Computer Graphics Laboratory
Scanner: Cyberware 3030 MS
Number of scans: 10
Total size of scans: 362,272 points (about 725,000 triangles)
Reconstruction: zipper
Size of reconstruction: 35947 vertices, 69451 triangles
Comments: contains 5 holes in the bottom

Range data + zippered reconstruction:

### Drill bit
Source: Stanford University Computer Graphics Laboratory
Scanner: Cyberware 3030 MS
Number of scans: 12
Total size of scans: 50643 points (about 101,000 triangles)
Reconstruction: zipper and vrip
Size of zippered reconstruction: 881 vertices, 1288 triangles
Size of vripped reconstruction: 1961 vertices, 3855 triangles

Range data + zippered and vripped reconstructions:

### Happy Buddha
Source: Stanford University Computer Graphics Laboratory
Scanner: Cyberware 3030 MS + spacetime analysis
Number of scans: ~60
Total size of scans: 4,586,124 points (about 9,200,000 triangles)
Reconstruction: vrip (conservatively decimated)
Size of reconstruction: 543,652 vertices, 1,087,716 triangles
Comments: hole-free, but contains small bridges due to space carving, so its topological genus is larger than it appears 

### Dragon
Source: Stanford University Computer Graphics Laboratory
Scanner: Cyberware 3030 MS + spacetime analysis
Number of scans: ~70
Total size of scans: 2,748,318 points (about 5,500,000 triangles)
Reconstruction: vrip (conservatively decimated)
Size of reconstruction: 566,098 vertices, 1,132,830 triangles
Comments: contains numerous small holes 

### Armadillo
Source: Stanford University Computer Graphics Laboratory
Scanner: Cyberware 3030 MS
Number of scans: 114 (but only 60-70 were used in vripped model)
Total size of scans: 3,390,515 points (about 7,500,000 triangles)
Reconstruction: vrip (conservatively decimated)
Size of reconstruction: 345,944 triangles

### Lucy
Source: Stanford University Computer Graphics Laboratory
Scanner: Stanford Large Statue Scanner
Number of scans: 47
Total size of scans: 58,241,932 points (approx 116 million triangles)
Reconstruction: vrip at 0.5 mm, holefilling
Size of reconstruction: 14,027,872 vertices, 28,055,742 triangles
Comments: hole-free, but contains small bridges due to space carving, so its topological genus is larger than it appears.
It may also have a few topological problems, making it not a proper manifold. Thanks to the Chaos Group for the rendering above. 

 *Note about this range dataset*: Lucy was scanned on two separate occasions. The raw range data (lucy_scans.tar.gz) and the VRIPped reconstruction (lucy.tar.gz) unfortunately do not correspond to the same scan of the statue. Moreover, the raw range data was never aligned, so the *.xf transform files in lucy_scans.tar.gz (as well as those in lucysd.tar.gz in the same directory) do not register the scans together. 

#### .SD format
http://graphics.stanford.edu/software/scanalyze/



## XYZ RGB models
Sólo se cuenta con la reconstrucción
No se usarán
