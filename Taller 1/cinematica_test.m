
[puma,pumap]=puma_param();
[q,qd,qdd]=joint_traj_puma560();

%Implementacion de la funcion homogeneus_matrix para el punto2:
[pts n]=size(q);
for j=1:pts
    for i=1:n
        [r,p]=homogeneus_matrix(puma.links(i),q(j,i));
    end
end


%Implementacion de la funcion forward_kinematics para el punto3:
%T=forward_kinematics(puma,q(15,:))    %Debbuging
for a=1:pts
    Ttt=forward_kinematics(puma,q(a,:));
end


%Implementacion de la funcion plot_traj para graficar la trayectoria del
%efector final del robot
[x,y,z]=plot_traj(puma,q);

