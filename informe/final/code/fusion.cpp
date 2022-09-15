double umbral = 1.5 * resolucion;
auto result = fusionar(registered, umbral);

double longitud_maxima_de_arista = 5 * resolucion;
auto tmesh = triangulate_3d(
    result.get_cloud(),
    5 * resolution
);

write_cloud_ply(*result.get_cloud(), "bunny_fusion.ply");
write_polygons(
    *result.get_cloud(),
    tmesh,
    "bunny_fusion.polygon"
);
// formato de PCL, pierde información de normales
auto polygon_mesh = tmesh_to_polygon(
    *result.get_cloud(),
    tmesh
);
pcl::io::savePolygonFilePLY(
    "bunny_fusion_pmesh.ply",
    polygon_mesh,
    false
);
