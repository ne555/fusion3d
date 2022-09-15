alignment alineacion_inicial;
alineacion_inicial.set_sample_ratio(0.25);
alineacion_inicial.set_feature_radius(6);
alineacion_inicial.set_y_threshold(4);
alineacion_inicial.set_axis_threshold(10 * M_PI / 180);

// source y target son dos nubes
// a las que se les realizó el preproceso
auto transformacion = alineacion_inicial.align(
    source,
    target
);
source.set_transformation(transformacion);
