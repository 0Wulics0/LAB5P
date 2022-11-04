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
### Resultados

#### Cojer marcador

https://youtu.be/NMbualhkjJg

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/200025472-2b09fdee-b9cf-4c65-ac82-6c2e4c5cea00.png" align="center" width="300"></a>

#### Dejar marcador

https://youtu.be/Vsqh0AElT7w

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/200027998-a3915ca7-52c2-477b-a57f-a90f31723417.png" align="center" width="300"></a>



#### Area de trabajo - Arcos

https://youtu.be/GAnZmhyxb7Y

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/200025330-3c7329ad-723b-4679-afe8-1711672faadd.png" align="center" width="300"></a>


#### Circulo:

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/199257976-5ebed1d9-9e56-4b7b-afc1-40b2eaee007d.png" align="center" width="300"></a>
<a href="url"><img src="[https://user-images.githubusercontent.com/37639887/199257976-5ebed1d9-9e56-4b7b-afc1-40b2eaee007d.png](https://user-images.githubusercontent.com/37639887/199275820-6bf9ce4f-b968-4602-a50b-2f30c12d7399.png)" align="left" width="200"></a>
<a href="url"><img src="https://user-images.githubusercontent.com/37639887/199275820-6bf9ce4f-b968-4602-a50b-2f30c12d7399.png" align="center" width="300"></a>

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/199260755-5ebc99d6-d986-403d-a220-bdb5f90a794f.png" align="center" width="300"></a>


#### Dibujo libre:

https://vimeo.com/766188835
https://vimeo.com/765787944/217f9d7aa3


<a href="url"><img src="https://user-images.githubusercontent.com/37639887/199278992-221b0d9d-c376-41e2-a818-15a14539d1af.png" align="center" width="300"></a>

![imagen](https://user-images.githubusercontent.com/37639887/199278696-05ff639b-c6fc-45bf-875b-76047331ebe2.png)

#### 5 puntos equidistantes:

https://vimeo.com/766193604/0d1a37a9da

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/199302975-d9df2527-5c06-4a43-abc4-1566efbe6d49.png" align="center" width="300"></a>

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/199863002-8114f6a2-5d66-4a45-92a1-7dd2acfb256c.png" align="center" width="300"></a>

#### Triangulo:

https://youtu.be/AzzmrknuZgA

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/199311543-c28bedf7-407c-419f-bd75-0581c7428adc.png" align="center" width="300"></a>


#### Letras:

https://vimeo.com/766188835

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/199885531-aece1442-4679-438d-8074-366dcd74a5ee.png" align="center" width="300"></a>

<a href="url"><img src="https://user-images.githubusercontent.com/37639887/199312702-ec272df2-1030-4ad3-b973-ea523c3aca38.png" align="center" width="300"></a>

#### 3 Lineas rectas

![imagen](https://user-images.githubusercontent.com/37639887/199322486-c721b2a7-8bcd-4959-89ab-e88b62b7d48c.png)

### Verificacion y analisis dimensional



#### Arcos:

Calidad del trazo:
Se observa una buena calidad del trazo en el arco exterior. En el arco interior, el robot cambia e altura, por lo que el trazo no es continuo. Al hacer solo el movimiento de la articulacion 1, este trazo es muy uniforme.

Radio (curvatura):
La curvatrua es perfecta por que solo se esta moviendo una articulacion.

Homogeneidad de todos los trazos: 
Hay total homogeneidad en el arco externo. En el interno se observa que no pinta aveces, pero en cuanto al trazo hecho, es muy homogeneo. 

Error porcentual = 0.2%

#### Circulo:
 
Calidad del trazo:
La calidad del trazo en este caso fue buena porque hubo contacto constante de la punta del marcador con el suelo y el robot no causo que la punta cambiara de orientacion respecto al piso.
 
Radio (curvatura):
Se habla de curvatura en vez de rectitud por ser un circulo. Se obseva que la curvatura no es constante. En particular se observa que hay menor calidad de la curvatura cuando la distancia entre puntos a moverse disminuye. Esta disminucion de calidad se ve como trazos poco uniformes con irregularidades puntiagudas y mayor desviacion general respecto a la curvatura solicitada. La razon de que sean mas erraticos es que hay mas paradas cuando los puntos sobre la trayecotria estan mas cerca, y a cada parada se genera un pico. Como la trayectoria tiene menos puntos en otros costados del circulo, la trayectoria tiene menos paradas y por lo tanto el trazo es mas uniforme. A parte de esto, la fuerza transversal ejercida sobre el marcador es mayor y esto hace que en cada moviemiento el marcador se pueda vencer la fuerza de friccion estatica con el tablero. Por esto es mas frecuente que el marcador no se mueva cuando los trazos son cortos.

Homogeneidad de todos los trazos: 

El trazo es homogeneo en intensidad, sin embargo cambia amedida que el marcador cambia su trayectoria, pues la punta de este es cuadrada, por lo que comienza con un trazo un poco mas delgado.


Como medida para definir la presicion del trazo se decide crear la siguiente medida:

```Error porcentual = (maxima distancia entre el trazo y la trayectoria real / maxima longitud del dibujo) x 100 ```

Para el circulo se tubo un error porcentual de 16%.


#### Dibujo libre:

Calidad del trazo:
Al principo no hay trao porque el marcador no se logra ubicar correctamente sobre el tablero. Despues de esto se obtiene un trazo uniforme pero se notan sobre el unos segmentos (eso se ve en la porcion que corresponde al medio criculo) que vienen de los pequenos pasos con los que se hizo el trazo. Cuando va a dibujar la siguiente mitad del semi ciruclo, el comportamiento anterior cammbia y se tiene una fuerte discontinuidad cuadrada producto. Estas se deben a que el robot, al ser controlado por Matlab, no hace el cambio de sus angulos simulataneamente para todas las articulaciones, sino que lo hace una a una, causando que en el tablero queden plasmados los trazos correspondientes a cuando se movio cada articulacion.
Finalmente se ve un comportamiento uniforme en el trazo final recto. Una vez termina este trayecto, el robot comienza a alejarse y este cambio se ve en el punto de deflexion de esta linea recta. Esto sucede porque el robot sigue en contacto con el talbero al comenzar a alejarse por que sus articulaciones se deflectan por ser de plastico y deben soportar el peso del robot. Este fenomeno lo llamaremos trazo errado por deflexion.

Rectitud:
En los trazos final e inicial rectos se ve gran rectitud. Principalmente en el ultimo, antes de la deflexion antes mencionada. En el inicial, el tramo que se pudo dibujar es corto y recto pero se puede ver que antes de este trazo el robot hace una trayectoria curva que no pinta. Esta hubiese sido otro trazo errado por deflexion durante la aproximacion del robot al tablero.

Radio: 
Evidentemente se puede ver que la primera mitad del semicirculo se dibujo como una recta. Esto sucede porque en esta ocasion el marcador estaba muy presionado contra el tablero, lo cual hizo que cuando se movian las articulaciones, los eslabones se deflectaran por la presion contra el piso, y por esto no pudo seguir la trayectoria curva.
En la mitad del circulo, el robot logra hacer la curvatura.
En la segunda mitad del circulo, el robot no logra hacerla por el mismo efecto y por el trazo puntiagudo explicado anteriormente.

Homogeneidad de todos los trazos: 
Se ve el mismo efecto explicado antes causado por la forma rectangular de la punta del marcador y ademas se tiene que al principio no se dibuja porque el marcador no suelta tinta debido a un residuo sobre el tablero.

Error porcentual = 6.6%

#### 5 puntos equidistantes:

Calidad del trazo:
Es buena, solo se observa el efecto de trazo errado por deflexion.

Rectitud:
No deberia dibujarse una linea en este caso, pero por el efecto mencionado anteriormente, se generan las lineas rectas cuando el robot se aleja de la posicion.

Homogeneidad de todos los trazos: 
Esta vez los trazos son totalmente homogeneos. solo difieren en la longitud por deflexiones distintas en distintas poses.

Error porcentual = 12.7%

#### Triangulo:

Calidad del trazo:
Se observa que el trazo tiene el efecto discreto del que se hablo antes por la comunicacion con matlab. Este se engrandece porque el numero de pasos utilizado fue mucho menor al utilizado antes (20).

Rectitud:
En promedio, el trazo es muy recto, como se ve por la trayectoria aproximada. Los trazos individuales reales tambien son muy rectos debido a los movimientos discretos del robot.

Homogeneidad de todos los trazos: 
Dos aristas tienen trazos con patrones muy similares escalonados. La otra tiende a tener trazos mas rectos. Esto sucde por la orientacion de las aristias respecto al robot. La arista mas recta se encontraba de manera que principalmente solo se debiera mover 1a articulacion 1 para dibujarla. Para los trazos escalonados se debieron utilizar 2, y su moviemiento discreto ocasiono el escalonamiento.

Error porcentual = 7.7%

#### Letras:

Calidad del trazo:
En este punto el marcador ha gastado mucha tinta y se comienza a desvanecer. El trazo es segmentado y curvo en ocasiones. Esta desviacion se da por la deflexion de las articulaciones al doblar por una esquina y al hacer el movimiento de alejarse del tablero.

Rectitud:
Se aprecia recitud en la mayoria de los trazos pero se ve afectada por la deflexion de nuevo. En los casos mas severos, se ve una curva en vez de una recta.

Homogeneidad de todos los trazos: 
Hay homogeneidad en la intensidad y tipo de errores en la escritura de ambas letras.

Error porcentual = 10.98%

#### 3 Lineas rectas:

Calidad del trazo:


Rectitud:
Solo en el trazo de mas inferior y superior se observa rectitud. En los demas se ve desvioacion debido a la deflexion.

Homogeneidad de todos los trazos: 
Los trazos se hacen con similar intensidad y ancho. La heterogeneidad viene de las trayectorias curvas ocasiondas por la flexibilidad del robot.

Error porcentual = 5.8%

### Ejecución de todos los métodos por script principal

Se ejecutan todos los métodos de dibujo disponibles y se verifica que no se ejecuten y se informe si se está o no sosteniendo el marcador.
[Ejecución](https://drive.google.com/file/d/1UGyeq4VhDznNy77IzjoUxrAqAJl0bPfN/view?usp=sharing)

### Conclusiones

Ros, junto a modelos de cinematica inversa y directa, permite controlar los movimientos de un robot por medio de python y/o Matlab con suficiente precision como para recorrer trayectorias complejas.

Matlab facilita la manipulacion del robot por medio de permitir el envio de comandos en ROS, creacion de trayectorias y simulacion de movimientos del robot dentro de su lenguaje. Esto hace que no requiramos al robot para verificar que la trayectoria programada sera correctamente ejecutada y tambien disminuye el riesgo que representa probar un programa ciegamente en el robot, pues este puede hacer ocasionar que realice movimientos bruscos y se dane.

La precision con la que el robot se comporte a comparacion del comportamiento simulado ideal depende de factores fisicos del robot, como la rigidez de sus eslabones, las fuerzas externas asociadas al proceso y la interfaz que lo controla (esto ultimo respecto al envio de valores de articulacion discontinuo desde Matlab). Esta precisision determina la calidad con la que lleva a cabo su funcion, pues como se evidencio, la desviacion entre trayectorias ideales y reales puede ser significativa.
