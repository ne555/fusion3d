def triangulate(cloud):
    xy = [p.z=0 in cloud]
    return delaunay(xy)

def submuestreo(cloud, n=2):
    #puntos que sobreviven cloud.size/n^2
    resolution = get_x_separation(cloud)
    return voxel_grid([n*resolution])

def good_points(cloud, threshold):
    #threshold1 = longitud excesiva de aristas (en relación al submuestreo)
    #threshold2 = cercanía a los bordes (en relación al submuestreo)
    #threshold3 = ángulo de ortogonalidad ~90
    #Buenos valores:
    #threshold = [3, 1.5, 80 (dot=0.2)]

    resolution = get_x_separation(cloud)
    t = triangulate(cloud)

    boundary = [for p in t if is_boundary(p)]
    big_edges += [for p in t if any(length(p.edge) > threshold1*resolution)]
    near_death += [for p in t if distance(p, boundary+big_edges) < threshold2*resolution]
    ortho = [for p in t if angle( normal(p), [0, 0, 1] ) > threshold3 ]

    bad_points = boundary + big_edges + near_death + ortho
    #return filter_out(cloud, bad_points)
    return bad_points
