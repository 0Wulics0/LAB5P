# Laboratorio 5 - Cinemática inversa del Phantom X Pincher
## Jorge Andrés Daza Rodríguez

## Daniel Melo

## Camilo Andrés Martín Moreno
### Caracterización del manipulador
Inicialmente es necesario dimensionar el manipulador a fin de determinar las longitudes de sus eslabones y ángulos entre sus articulaciones para
obtener los parámetros DH y de esta manera tener la capacidad de realizar las simulaciones correspondientes a todas las trayectorias recorridas por el
manipulador y de la misma manera poder determinar la cinemática inversa del mismo. Con este fin se establecen los siguientes marcos y las siguientes 
longitudes de eslabón:

![Diagrama](/Img/Diagrama.jpeg)

![MarcosDH](/Img/MarcosDH.jpeg)

Con estos parámetros se obtienen los siguientes parámetros DH:

![Parámetros DH](/Img/DH.png)

Es necesario considerar que el marcador se va a sujetar de forma perpendicular a la herramienta del manipulador, por lo que este último ángulo simpre 
corresponderá al negativo de la suma de los dos ángulos anteriores, logrando así que el marcador se encuentre perpenticular a la superficie de dibujo
en todo momento. Teniendo esta consideración en mente se establece la cinemática inversa del manipulador la cual parte de las componentes 'x' y 'y'
de una pose deseada.

### Cinemática inversa

```octave
th1_m = atan2(-y_p,-x_p);
th2_do = alpha - beta; %Codo abajo
th2_do = (pi/2+th2_do);
th2_up = alpha + beta; %Codo arriba
th2_up = (pi/2+th2_up);
th3_up = atan2(real(sin_th3),cos_th3);
th3_do = -th3_up; %Codo abajo
th4_do = atan2(-HT3_do(1,1),HT3_do(2,1));
```

Se puede observar que para los ángulos de las articulaciones 2 y 3 pueden tener dos soluciones, correspondientes a la solución codo arriba y la solución
codo abajo y en este caso se usarán las soluciones codo abajo. Por la forma en la que está definida la articulación 4, al momento de dibujar sobre el 
tablero, esta siempre tendrá la solución codo abajo.

### Envío de mensajes ROS-MATLAB
Teniendo la cinemática inversa, se debe realizar una función que permita enviar una configuración de ángulos obtenidos de la cinemática inversa a los
motores dynamixel del Pincher, para lo cual se desarrolla la siguiente función:

```octave
function movePX(msg,cliente,thetas,mov_gripper)
    if mov_gripper
        msg.AddrName = "Goal_Position";
        msg.Id = 5;
        msg.Value = mapfun(rad2deg(thetas),-150,150,0,1023);
        call(cliente,msg);
    else
        msg.AddrName = "Goal_Position";
        %pause(0.1);
        q = rad2deg(thetas(2,:));
    %     q(5) = o_gripper;
    for i=4:-1:1
        msg.Id = i;
        msg.Value = mapfun(q(i),-150,150,0,1023);
        call(cliente,msg);
    end
    end
end
```

Además de enviar un mensaje con la configuración deseada, utiliza al final un parámetro booleano para diferenciar entre el envío de una postura determinada
y la apertura o cierre del gripper del manipulador. Adicionalmente se apoya en otra función la cual se encarga de recibir e iterar sobre una matriz 
multidimensional que contiene todas las poses requeridas que serán enviadas a la función de cinemática inversa para posteriormente enviar el mensaje a ROS.
Esta función también se encarga de mostrar iterativamente los movimientos realizados por el manipulador mediante el toolboz de PeterCorke.

```octave
function move(Mov,Robot,l,msg,cliente)
     sz = size(Mov);
     figure(1)
     for i=1:sz(3)
     thetas = InverseKinematics(Robot,l,Mov(:,:,i));
     movePX(msg,cliente,thetas, false);
     hold on;
     trplot(eye(4),'rgb','arrow','length',25,'frame','or')
     hold on;
     plot3(Mov(1,4,i),Mov(2,4,i),Mov(3,4,i),'ro')
     hold on;
     xlim([-20,20])
     ylim([-20,20])
     Robot.plot(thetas(2,:),'notiles','noname')
     end 
end
```

### Trayectorias programadas

### Ejecución de las trayectorias programadas
El archivo Main del código se encarga de 3 tareas principales. Establece la conexión con el nodo principal de ros y genera un mensaje mediante un servicio
y un suscriptor para obtener datos acerca del estado de los motores. Usando los parámetros determinados anteriormente utiliza el toolbox de PeterCorke para
generar un manipulador. Finalmente ejecuta continuamente el código que le pedirá al usuario por el método que desea ejecutar. Dentro de cada una de las
funciones de generación de trayectorias se considera si ya se tiene sujetado el marcador mediante el parámetro "Expo".

```octave
rosshutdown;
rosinit;
%%
clear;
clf("reset")

cliente = rossvcclient('/dynamixel_workbench/dynamixel_command');
msg = rosmessage(cliente); %Creación de mensaje
Sub=rossubscriber('/dynamixel_workbench/joint_states');


l = [13.27 10.3 10.3 6.57]; % Longitudes eslabones

l_T = 5.5;
% Definicion del robot RTB
L(1) = Link('revolute','alpha',-pi/2,'a',0,      'd',l(1),'offset',0);
L(2) = Link('revolute','alpha',0,    'a',l(2),   'd',0,   'offset',-pi/2);%,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,    'a',l(3),   'd',0,   'offset',0);%,    'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,    'a',l(4),   'd',0,   'offset',0);

Robot = SerialLink(L,'name','Px');

Robot.tool = [0 0 1 0;
              1 0 0 0;
              0 1 0 0;
              0 0 0 1];
expo=0;

while(true)
metodo=input("Ingrese el método deseado \n",'s')
switch metodo
    case 'm'
[expo,z]=marcador(msg,Sub,cliente,Robot,l,expo);
    case 'c'
circulo(msg,Sub,cliente,Robot,l,expo);
    case 'l'
lineasParalelas(msg,Sub,cliente,Robot,l,expo);
    case 'curv'
dibujoLibre(msg,Sub,cliente,Robot,l);
    case 'let'
letras(msg,Sub,cliente,Robot,l, expo);
    case 'p'
end
end
```

### Ejemplos de funcionamiento

### Verificacion dimensional
![imagen](https://user-images.githubusercontent.com/37639887/199257976-5ebed1d9-9e56-4b7b-afc1-40b2eaee007d.png  =250x250 )
![imagen](https://user-images.githubusercontent.com/37639887/199275820-6bf9ce4f-b968-4602-a50b-2f30c12d7399.png | width=100)
![imagen](https://user-images.githubusercontent.com/37639887/199260755-5ebc99d6-d986-403d-a220-bdb5f90a794f.png | width=100)
Calidad del trazo:
La calidad del trazo en este caso fue buena porque hubo contacto constante de la punta del marcador con el suelo y el robot no causo que la punta cambiara de orientacion respecto al piso.
Rectitud ():

Se habla de curvatura en vez de rectitud por ser un circulo. Se obseva que la curvatura no es constante. En particular se observa que hay menor calidad de la curvatura cuando la distancia entre puntos a moverse disminuye. Esta disminucion de calidad se ve como trazos poco uniformes con irregularidades puntiagudas y mayor desviacion general respecto a la curvatura solicitada. La razon de que sean mas erraticos es que hay mas paradas cuando los puntos sobre la trayecotria estan mas cerca, y a cada parada se genera un pico. Como la trayectoria tiene menos puntos en otros costados del circulo, la trayectoria tiene menos paradas y por lo tanto el trazo es mas uniforme. A parte de esto, la fuerza transversal ejercida sobre el marcador es mayor y esto hace que en cada moviemiento el marcador se pueda vencer la fuerza de friccion estatica con el tablero. Por esto es mas frecuente que el marcador no se mueva cuando los trazos son cortos.
Radio:
Homogeneidad de todos los trazos:

