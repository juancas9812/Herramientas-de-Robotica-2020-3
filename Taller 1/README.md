# CINEMÁTICA DIERCTA E INVERSA

Los codigos de la carpeta son parte del procedimiento para hallar la cinemática directa e inversa de un robot manipulador. 

## Forward Kinematics/Cinemática Directa
Primero se formulan las funciones para hallar la cinemática directa: homogeneous_matrix, y forward_kinematics.

* homogeneous_matrix:

>Se utiliza la matriz DH del robot junto con la posicion de una de las articulaciones para encontrar la matriz de transformacion homogenea
(matriz de rotación y vector de posicion)

* forward_kinematics:

>Para una posicion del robot, se utiliza la funcion homogeneous matrix el numero de veces igual al numero de articulaciones del robot para encontrar 
la posicion del efector final

## Inverse Kinematics/Cinemática Inversa
Se busca con la cinemática inversa, a partir de una posicion del robot, encontrar la siguiente posicion. Se dan entonces las funciones Jacobian e ikine. 
(ikine todavia no esta completa)

* Jacobian: 
>A partir de una posicion del robot se encuentra el jacobiano para establecer las posiciones, velocidades y aceleraciones de las articulaciones del robot.
