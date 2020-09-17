
%Esta funcion contiene la solucion del segundo punto del taller 1- parte 1,
%en donde se pide calcular las matrices de transformacion homogenea para 
%toda la estructura del robot

%Salidas:
%r: Matriz R de la matriz A
%p: Vector P de la matriz A

%Entradas:
%DH: parametros de la articulacion (link) asociada a la matriz D-H que
%    define al robot (se utiliza dentro de la funcion las propiedades de la 
%    clase Link del toolbox de robotica)
%q_i: posicion articular (viene del vector q)

function [r,p]=homogeneus_matrix(DH, q_i)

if(DH.offset==0)     %Se comprueba si es rotacional o prismatico
    theta=q_i;     %En caso de que sea rotacional, el angulo de rotacion pasa a ser el valor de q
    
    %Se hace el calculo de la matriz homogenea para la articulacion
    MTH=[cos(theta) -cos(DH.alpha)*sin(theta)  sin(DH.alpha)*sin(theta) DH.a*cos(theta);
         sin(theta)  cos(DH.alpha)*cos(theta) -sin(DH.alpha)*cos(theta) DH.a*sin(theta);
                  0             sin(DH.alpha)             cos(DH.alpha)            DH.d;
                  0                         0                         0               1 ];
else
    d=q_i;        %En caso de que sea prismatico, la distancia de translacion pasa a ser el valor de q
    MTH=[cos(DH.theta) -cos(DH.alpha)*sin(DH.theta)  sin(DH.alpha)*sin(DH.theta) 0;
         sin(DH.theta)  cos(DH.alpha)*cos(DH.theta) -sin(DH.alpha)*cos(DH.theta) 0;
                     0                sin(DH.alpha)                cos(DH.alpha) d;
                     0                            0                            0 1 ];
end

%Valores a retornar:
r=MTH(1:3,1:3);
p=MTH(1:3,4);

end


