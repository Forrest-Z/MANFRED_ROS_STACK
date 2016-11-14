function nuevo_gen = cruce(gen1, gen2)

posicion1 = ceil(rand(1,1)*length(gen1));
posicion2 = ceil(rand(1,1)*length(gen1));

p1 = min(posicion1, posicion2);
p2 = max(posicion1, posicion2);

nuevo_gen = zeros(size(gen1));

subcadena1 = gen1(p1:p2);
nuevo_gen(p1:p2) = subcadena1;
[~, posiciones] = setdiff(gen2, subcadena1);
subcadena2 = gen2(sort(posiciones));

if p1 > 1
    nuevo_gen(1:p1-1) = subcadena2(1:p1-1);
end;

if p2 < length(gen1)
    nuevo_gen(p2+1:end) = subcadena2(p1:end);
end;

