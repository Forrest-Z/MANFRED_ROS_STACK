function elemento = mutacion(elemento, iguales)

prob_mutacion = min(1, iguales/1000);
aleatorio = rand(1,1);

if aleatorio < prob_mutacion
    aleatorio = rand(1,1);
    if aleatorio < 0.5
        posicion1 = ceil(rand(1,1)*length(elemento));
        posicion2 = ceil(rand(1,1)*length(elemento));
        p1 = min(posicion1, posicion2);
        p2 = max(posicion1, posicion2);
        elemento(p1:p2) = elemento(p2:-1:p1);
    else
        posicion1 = ceil(rand(1,1)*length(elemento));
        posicion2 = ceil(rand(1,1)*length(elemento));
        ciudad = elemento(posicion1);
        if posicion1 < length(elemento)
            if posicion1 > 1
                elemento = [elemento(1:posicion1-1) elemento(posicion1+1:end)];
            else elemento = elemento(2:end);
            end;
        else
            elemento = elemento(1:end-1);
        end;
        if posicion2 < length(elemento)
            if posicion2 > 1
            elemento = [elemento(1:posicion2-1) ciudad elemento(posicion2:end)];
            else
                elemento = [ciudad elemento];
            end;
        else
                elemento = [elemento ciudad];
        end;  
    end;
end;
