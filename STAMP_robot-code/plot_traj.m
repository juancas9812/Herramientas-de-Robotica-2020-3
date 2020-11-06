%Esta funcion traza una grafica de la trayectoria de la posicion del
%efector final de un robot

%Salidas:
%x:vector con todas las componentes en x de los puntos de la trayectoria
%y:vector con todas las componentes en y de los puntos de la trayectoria
%z:vector con todas las componentes en z de los puntos de la trayectoria

%Entradas:
%rob: matriz DH del robot
%q: puntos de trayectoria

function [x,y,z]=plot_traj(rob,q)
[pts n]=size(q);
T=forward_kinematics(rob,q(1,:));
x=T(1,4);
y=T(2,4);
z=T(3,4);
for i=2:pts
    T=forward_kinematics(rob,q(i,:));
    x=[x,T(1,4)];
    y=[y,T(2,4)];
    z=[z,T(3,4)];
end

%plt=plot3(x,y,z,'- .r');
%plt.LineWidth=2;
%plt.MarkerSize=20;
%xlabel("X[m]");
%ylabel("Y[m]");
%zlabel("Z[m]");
%axis([0 0.5 -0.2 0.2 0 1]);
%title("Trayectoria del efector final del robot(Vista en 3D)");
%grid on;

end
