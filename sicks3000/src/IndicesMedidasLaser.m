clear all;
close all
clc;

format long
% Constantes del laser sick s3000
% Arco total abarcado (en grados).
ang_tot = 190;
% Arco mitad abarcado (en grados).
ang_tot_mitad = ang_tot/2;
% Maxima resolucion angular del laser.
res_ang = 0.25;
% Numero de medidas del laser a maxima resolucion, abarcando el angulo
% maximo.
num_med_laser_tot = (ang_tot/res_ang) + 1;

% Como se etiqueta los barridos.
% El barrido que sale del centro de laser se considera a angulo 0.
% A la derecha del centro los angulo se consideran negativos (recorrido horario), hasta -95
% grados (190/2).
% A la izquierda del centro los angulos se consideran positivos (recorrido antihorario), hasta 95
% grados (190/2).

% Es posible que no se quiera una resolucion angular tan pequenia, ya que de
% esa forma tenemos muchas medidas, 761.
% Si queremos un incremento angular entre medidas superior a 0'25 grados,
% pero ojo, siempre multimo entero de 0'25 grados, entonces el siguiente
% bloque de codigo es necesario. Con el siguiente bloque de codigo
% obtenemos desde que angulo y hasta que angulo podemos barrer usando una
% resolucion angular arbitraria (de nuevo recuerdo, multiplo entero de 0.25
% grados).

% Si por ejemplo queremos usar un incremento angular de 0'75 grados entre dos
% medidas consecutivas, resulta que solo podemos obtener 126 medidas a la
% derecha del cero y otras 126 a la izquierda del cero.
% 95/0'75 = 126'6667 --> 126 medidas.
% numero total de medidas 126 + 126 + 1 (la medida que se hace en cero) =
% 253.
% Con estas 126 medidas a cada lado, llegamos desde -94'5 grados (- 126 *
% 0'75) hasta 94'5 grados (126 * 0'75), es decir, se abarca un angulo de
% 189 grados.

% Coeficiente de incremento.
% CAMBIAR ESTE NUMERO SOLO
coef_incr = 3
% Incremento angular.
incr_ang = coef_incr * res_ang
incr_ang_rad = incr_ang/180 * pi
% Numero de sectores circulares mitad.
% nmulc = floor((ntml - 1)/(2*ci))
num_sec_circ_mitad = floor(ang_tot_mitad/incr_ang)
% Numero de medidas incluyendo la del cero. (numero de medidas laser)
num_sec_cir_tot = 2 * num_sec_circ_mitad
num_med_laser = num_sec_cir_tot + 1
% Angulo de finalizacion del barrido.
ang_fin = num_sec_circ_mitad * incr_ang
ang_fin * 2
ang_fin_rad = ang_fin/180 * pi
% Indice de medida inicial
% En el telegrama enviado desde el sick s3000 al pc en que medida se debe 
% comenzar para obtener el primer angulo de medida (angulo negativo)
% imi = ((ang_tot_mitad - ang_fin) / res_ang) + 1
% Simpificando: -->> 1/ra = 1/0.25 = 4


ind_ini_c = ((ang_tot_mitad - ang_fin) / res_ang)
ind_ini_matlab = ((ang_tot_mitad - ang_fin) / res_ang) + 1

ang_laser = -95:0.25:95;

% Indices que debo usar en C/C++
ind_med_c = ind_ini_c + (coef_incr * (0 : num_med_laser - 1))
ind_ini_matlab + (coef_incr * (0 : num_med_laser - 1))
%ang_medidos = ang_laser(ind_ini_matlab) : incr_ang : ang_tot_mitad
ang_medidos = ang_laser(ind_ini_matlab + (coef_incr * (0 : num_med_laser - 1)))
% ESTO ES SOLO PARA PINTAR.
% EN AZUL TODAS LAS MEDIDAS QUE DA EL LASER.
close all;
ang_laser = 90 + ang_laser; 
ang_laser_rad = (ang_laser ./180) .* pi;
c = cos(ang_laser_rad);
s = sin(ang_laser_rad);
plot(c, s);
grid on;
line([0 c(1)], [0, s(1)])
line([0 c(end)], [0, s(end)])

hold on
% EN ROJO LAS MEDIDAS QUE ESCOGEMOS.
ang_medidos = 90 + ang_medidos; 
ang_medidos_rad = (ang_medidos ./180) .* pi;
c = cos(ang_medidos_rad);
s = sin(ang_medidos_rad);

plot( c(1), s(1), 'r') 

for i=1:length(ang_medidos)
line([0 c(i)], [0, s(i)], 'Color', 'r') 
end

title('Medidas proporcionadas por el laser (azul) y medidas escogidas (en rojo)')