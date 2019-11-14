def triangulate(cloud):
    xy = [p.z=0 in cloud]
    return delaunay(xy)

def good_points(cloud, submuestreo, threshold):
    submuestreo = {1..n} cantidad de puntos en celda para submuestrear
        (puntos que sobreviven n/sub^2)
    threshold1 = longitud excesiva de aristas (en relación al submuestreo)
    threshold2 = cercanía a los bordes (en relación al submuestreo)
    threshold3 = ángulo de ortogonalidad ~90

    resolution = get_x_separation(cloud)
    cloud = voxel_grid([submuestreo*resolution])
    t = triangulate(cloud)

    boundary = [for p in t if is_boundary(p)]
    big_edges += [for p in t if any(length(p.edge) > threshold1*resolution)]
    near_death += [for p in t if distance(p, boundary+big_edges) < threshold2*resolution]
    ortho = [for p in t if angle( normal(p), [0, 0, 1] ) > threshold3 ]

    bad_points = boundary + big_edges + near_death + ortho
    #return filter_out(cloud, bad_points)
    return cloud, bad_points
