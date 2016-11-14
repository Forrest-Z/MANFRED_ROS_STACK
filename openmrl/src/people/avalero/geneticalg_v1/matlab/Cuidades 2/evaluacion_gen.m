function puntuacion=evaluacion_gen(gen, posiciones)

orden = gen;

puntuacion = 0;
for i=2:length(orden)
    puntuacion = puntuacion + sqrt((posiciones(orden(i),1)-posiciones(orden(i-1),1))^2+(posiciones(orden(i),2)-posiciones(orden(i-1),2))^2);
end;

puntuacion = puntuacion + sqrt((posiciones(orden(end),1)-posiciones(orden(1),1))^2+(posiciones(orden(end),2)-posiciones(orden(1),2))^2);