
%Esta funcion contiene parte de la solucion del primer punto del taller 1- 
%parte 2, en donde se calcula el jacobiano para la posicion articular dada

%Salidas:
%J: Matriz del jacobiano para la posicion articular

%Entradas:
%DH: parametros de la articulacion (link) asociada a la matriz D-H que
%    define al robot (se utiliza dentro de la funcion las propiedades de la 
%    clase Link del toolbox de robotica)
%q: posiciones articulares (vector q)

function J=Jacobian(DH,q)
Tv=struct('r',{},'p',{});
ZP=struct('z',{},'p',{});
[n1,n2]=size(q);
[r,p]=homogeneus_matrix(DH.links(1),q(1));    %Se obtiene la primera matriz homogenea
T=[r p];
Tv(1).r=r      %Para calcular el jacobiano no se necesita la matriz de 4*4 se necesita solo 
              %la rotacion y el desplazamiento
Tv(1).p=p
%Procedimiento similar al de forward kinematics, se guarda cada iteracion
%para cada articulacion
for ind=2:n2          
    [r,p] = homogeneus_matrix(DH.links(ind),q(ind));
    A = [r p];
    A = [A;0 0 0 1];
    T = T*A;
    Tv(ind).r = T(1:3,1:3);
    Tv(ind).p = T(1:3,4);   
end
Jaux=[0;0;1];
if (DH.links(1).offset==0)
    J=cross(Jaux,Tv(n2).p);
    J=[J;Jaux];
else
    J=[Jaux;[0;0;0]];
end
Jcol=[0;0;0;0;0;0];
for j=1:(n2-1)
    ZP(j).r=Tv(j).r(1:3,3);        %De los terminos del jacobiano independiente si es rotacional o no
                                  %se usa el termino A_0,0*[0;0;1]
    if(DH.links(j).offset==0)     %Se comprueba si es rotacional o prismatico
        Jcol(4:6)=ZP(j).r;
        ZP(j).r=cross(ZP(j).r,(Tv(n2).p-Tv(j).p));
    end
    Jcol(1:3)=ZP(j).r;
    J=[J Jcol];
    Jcol=[0;0;0;0;0;0];
%    theta=q_i;     %En caso de que sea rotacional, el angulo de rotacion pasa a ser el valor de q
end
end
