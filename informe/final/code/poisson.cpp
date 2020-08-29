auto nube = create<cloudnormal>();
pcl::io::loadPLYFile<pointnormal>("bunny_fusion.ply", *nube);

pcl::Poisson<pointnormal> poisson;
poisson.setInputCloud(nube);
pcl::PolygonMesh mesh;
poisson.setDepth(8);
poisson.reconstruct(mesh);
// al reconstruir se pierde la información de normales
pcl::io::savePolygonFilePLY("bunny_reconstructed.ply", mesh, false);
