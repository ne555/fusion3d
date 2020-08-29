auto source_icp = join_cloud_and_normal(source);
auto target_icp = join_cloud_and_normal(target);

//modifica source_icp
icp_correction(source_icp, target_icp, resolucion);
