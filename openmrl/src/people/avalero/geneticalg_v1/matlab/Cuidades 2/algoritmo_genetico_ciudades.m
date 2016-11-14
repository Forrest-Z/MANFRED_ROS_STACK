function algoritmo_genetico_ciudades(posiciones)

n_ciudades = size(posiciones,1);
n_individuos = 100;

poblacion = poblacion_incial(n_ciudades,n_individuos);
evolucion = [];

convergencia = 0;
iguales = 1;

while ~convergencia
    for i = 1:n_individuos
        puntuacion(i) = evaluacion_gen(poblacion(i,:), posiciones);
    end;
    
    [~, orden] = sort(puntuacion,'ascend');
    nueva_poblacion = zeros(size(poblacion));
    nueva_poblacion(1,:) = poblacion(orden(1),:);
    for i=2:n_individuos
        [p1, p2] = elementos_aleatorios(n_individuos);
        individuo1 = poblacion(orden(p1),:);
        individuo2 = poblacion(orden(p2),:);
        nueva_poblacion(i,:) = cruce(individuo1, individuo2);
        nueva_poblacion(i,:) = mutacion(nueva_poblacion(i,:),iguales);
    end;
    mejor_puntuacion = puntuacion(orden(1));
    evolucion(end+1) = mejor_puntuacion;
    
%     if mod(length(evolucion),100)==0;
%         subplot(1,2,1); plot(evolucion);
%         mejor_individuo = nueva_poblacion(1,:);
%         mejor_orden = mejor_individuo;
%         x_ordenado = posiciones(mejor_orden,1);
%         x_ordenado(end+1) = x_ordenado(1);
%         y_ordenado = posiciones(mejor_orden,2);
%         y_ordenado(end+1) = y_ordenado(1);
%         subplot(1,2,2); hold off; plot(x_ordenado, y_ordenado); hold on; plot(posiciones(:,1), posiciones(:,2), '*r'); axis equal;
%         pause(0.1);
%     end;
    
    poblacion = nueva_poblacion;
    
    if length(evolucion) > 1
        if evolucion(end) == evolucion(end-1);
            iguales = iguales+1;
        else
            iguales = 0;
        end
    end
    
    minimo = 100;
    if length(evolucion) < minimo
        %al menos 10 iteraciones
        convergencia = 0;
    else
        if evolucion(end-(minimo-1))/evolucion(end) > 1.00001
            %si en las 1000 últimas iteraciones no ha mejorado el 0.01%
            %paramos
            convergencia = 0;
        else
            convergencia = 1;
        end;
    end;
    
end

figure;
subplot(1,2,1); plot(evolucion);
mejor_individuo = nueva_poblacion(1,:);
mejor_orden = mejor_individuo;
x_ordenado = posiciones(mejor_orden,1);
x_ordenado(end+1) = x_ordenado(1);
y_ordenado = posiciones(mejor_orden,2);
y_ordenado(end+1) = y_ordenado(1);
subplot(1,2,2); hold off; plot(x_ordenado, y_ordenado); hold on; plot(posiciones(:,1), posiciones(:,2), '*r'); axis equal;
pause(0.1);
   