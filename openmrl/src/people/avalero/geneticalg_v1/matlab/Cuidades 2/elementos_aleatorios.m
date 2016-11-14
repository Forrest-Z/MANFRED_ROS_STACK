function [p1, p2] = elementos_aleatorios(n_elementos)

v = 1:n_elementos;
peso = 1./(1.05.^v);
peso = peso / sum(peso); %la suma de probabilidades es uno

cdf = cumsum(peso); %función de distribución acumulada

%genero números por el método de monte-carlo

aleatorio = rand(1,1); %numero entre 0 y 1
p1 = find (cdf >= aleatorio, 1, 'first');

aleatorio = rand(1,1);
p2 = find (cdf >= aleatorio, 1, 'first');