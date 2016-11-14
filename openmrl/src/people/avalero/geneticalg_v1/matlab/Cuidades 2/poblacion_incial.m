function poblacion = poblacion_incial(n_ciudades, n_elementos)

poblacion = rand(n_elementos, n_ciudades);
[~, poblacion] = sort(poblacion,2);