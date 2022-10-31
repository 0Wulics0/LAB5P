rosshutdown;
rosinit;
%%
clear;
clf("reset")

%cliente = rossvcclient('/dynamixel_workbench/dynamixel_command');
%msg = rosmessage(cliente); %Creación de mensaje
% 
%Sub=rossubscriber('/dynamixel_workbench/joint_states');


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
  

[expo,z]=marcador(msg,Sub,cliente,Robot,l);
lineasParalelas(msg,Sub,cliente,Robot,l,expo,z);
letras(msg,Sub,cliente,Robot,l, expo);
zabajo = -10
circulo(msg,Sub,cliente,Robot,l,zabajo);