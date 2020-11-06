
[stamp, stampparams]=stamp_param();
% %stamp.plot([0 -pi/4 pi/4 -pi/2 0]) [q,qd,qdd]=joint_traj_puma560();
% 
% q=traj_stamp();
% 
% %Implementacion de la funcion homogeneus_matrix para el punto2:
% [pts n]=size(q);
% for j=1:pts
%     for i=1:n
%         [r,p]=homogeneus_matrix(stamp.links(i),q(j,i));
%     end
% end
% 
% 
% %Implementacion de la funcion forward_kinematics para el punto3:
% %T=forward_kinematics(stamp,q(15,:))
% for a=1:pts
%     Ttt=forward_kinematics(stamp,q(a,:));
% end
% 
% 
% %Implementacion de la funcion plot_traj para graficar la trayectoria del
% %efector final del robot
% [x,y,z]=plot_traj(stamp,q(pts,:));
% 
% figure
% plot3(x,y,z,'Color','g')
% xlabel('X[m]')
% ylabel('Y[m]')
% zlabel('Z[m]')
% title('Robot STAMP y trayectoria')
% grid on
% 
% stamp.plot(q)
% stamp.teach(q)


 
T=forward_kinematics(stamp,[pi/2,-pi/4,pi/2,3*pi/4,0]);

Vk=0.3;
Ts=0.01;
pr=0.3;

L=[T(1,4) T(2,4) T(3,4);-0.3 0.65 0.15];
traja=traj_planner(1,L,Vk,pr,Ts,0);
[ptstraja,coltraja]=size(traja);

%Se crean los demas puntos de la trayectoria y se calculan los perfiles de la trayectoria
traj=[traja;task_traj(Vk,pr,Ts,traja(ptstraja,coltraja))];
[ptstraj, coltraj]=size(traj);

%Se guardan por aparte las posiciones,velocidades,aceleraciones y tiempo
%para graficar el perfil de la trayectoria
PosX=traj(:,1);PosY=traj(:,2);PosZ=traj(:,3);
VelX=traj(:,4);VelY=traj(:,5);VelZ=traj(:,6);
AceX=traj(:,7);AceY=traj(:,8);AceZ=traj(:,9);
tiempo=traj(:,10);

VelT=[];
for in2=1:ptstraj
    VelT=[VelT;norm([VelX(in2),VelY(in2),VelZ(in2)])];
end

AceT=0;
for in3=2:ptstraj
    AceT=[AceT;(VelT(in3)-VelT(in3-1))/Ts];
end

traj_xyz=traj(:,1:3);



qik=stamp.ikine(T,'q0',[pi/2,-pi/4,pi/2,3*pi/4,0],'mask',[1 1 1 1 1 0]);

%T_ik_fw=forward_kinematics(stamp,qik);
qiks=[];
X_ik_fw=[];Y_ik_fw=[];Z_ik_fw=[];
%q0=qik;

for in=1:ptstraj
    T=transl(PosX(in),PosY(in),PosZ(in))*trotx(-90,'deg');
    qik=stamp.ikine(T,'mask',[1 1 1 1 1 0],'u');
    %q0=qik;
    qiks=[qiks;qik];
    T_ik_fw=forward_kinematics(stamp,qik);
    X_ik_fw=[X_ik_fw;T_ik_fw(1,4)];
    Y_ik_fw=[Y_ik_fw;T_ik_fw(2,4)];
    Z_ik_fw=[Z_ik_fw;T_ik_fw(3,4)];
end


figure
plot3(X_ik_fw,Y_ik_fw,Z_ik_fw,'-k.')
xlabel('X[m]')
ylabel('Y[m]')
zlabel('Z[m]')
title('Robot STAMP y trayectoria')
grid on

stamp.plot(qiks)

%%

%Configuracion en un punto de la tarea del robot
q=[qiks(1,:);qiks(1,:)]
figure
plot3([0,0],[0,0],[0,0],'-b');
grid on
stamp.plot(q)
%%
%Configuracion en un punto de la tarea del robot
q=[qiks(300,:);qiks(300,:)]
figure
plot3([0,0],[0,0],[0,0],'-b');
grid on
stamp.plot(q)
%%
%Configuracion en un punto de la tarea del robot
q=[qiks(1280,:);qiks(1280,:)]
figure
plot3([0,0],[0,0],[0,0],'-b');
grid on
stamp.plot(q)
%%
q=[qiks(2000,:);qiks(2000,:)]
figure
plot3([0,0],[0,0],[0,0],'-b');
grid on
stamp.plot(q)
