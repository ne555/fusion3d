auto [cloud, mesh] = load_triangle_mesh(
    "bun_fusion.ply",
    "bun_fusion.polygon"
);
extract_biggest_connected_component(cloud, mesh);
auto boundary_points = boundary_points(mesh);
