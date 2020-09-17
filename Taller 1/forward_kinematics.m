
%Esta funcion contiene la solucion del tercer punto del taller 1- parte 1,
%en donde se pide calcular la solucion al problema de cinematica directa
%del robot

%Salida:
%T: matriz homogenea del efector final con respecto al origen o base del
%   robot.

%Entradas:
%DH: parametros DH del robot. (Es la matriz completa, NO es solo una
%    articulacion como pasa en la funcion homogeneus_matrix
%q: vector de posiciones/movimientos de las articulaciones (viene de la
%   matriz de las trayectorias)

function T=forward_kinematics(DH,q)
[n1,n2]=size(q);    %Se verifican los grados de libertad del robot 
[r,p]=homogeneus_matrix(DH.links(1),q(1));    %Se obtiene la primera matriz homogenea
T=[r p];      %Se juntan la matriz r y el vector p
T=[T;0 0 0 1];    %Se obtiene la primera matriz para multiplicar las matrices y obtener
                   %la relacion del efector final
%Se realizan las 3 instrucciones anteriores para cada articulacion y se 
%multiplican las matrices hasta obtener el resultado de T:
for ind=2:n2          
    [r,p]=homogeneus_matrix(DH.links(ind),q(ind));
    A=[r p];
    A=[A;0 0 0 1];
    T=T*A;
end
    
end