rosshutdown;
rosinit;
%%
clear;
clf("reset")

cliente = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creación de cliente de pose y posición
% msg = rosmessage(cliente); %Creación de mensaje
% 
% msg.AddrName = "Torque_Limit";
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


T0 =   Robot.fkine([0 0 0 0]);
n = 20; %Number of pose
%T1 = Robot.fkine([1.5544 0.7056 1.9226 0.3988]);

% T2 = 
% 
% T3 =
% 
% T4 =

% T1 = transl(2.5,-10,-sum(l) + l_T - 1)*T0*troty(pi); %Left
% Tt1 = transl(0,0,sum(l)/2 - l_T)*T1;
% 
% TFt = transl(14.5,-0.8,-sum(l) + 2*l_T - 1)*T0*troty(pi);
% 
% T2 = transl(14.5,-0.8,-sum(l) + l_T)*T0*troty(pi); %Front
% Tt2 = transl(0,0,sum(l)/2 - l_T)*T2;
% [0.8283, 1.5238, 0.5880, -0.5471, -1.3959]
% 
% T3 = transl(3.5,10,-sum(l) + l_T - 1)*T0*troty(pi);%Right
% Tt3 = transl(0,0,sum(l)/2 - l_T)*T3;
% 
% T4 = transl(14.5,-0.8,-sum(l) + l_T + 1)*T0*troty(pi); %Front
% Tt4 = Tt2;
T1=Robot.fkine([0.8283, 1.5238, 0.5880, -0.5471]);%, -1.3959])
T2=Robot.fkine([0.8283-pi/3, 1.5238, 0.5880, -0.5471]);%, -1.3959])
a=transl(0,0,30);
TinitCircInterior = Robot.fkine([-1.3039,   0.9971,    2.3726,   -1.8203])%,   -1.3908])
Mov = cat(3,  ctraj(T0,T1,n));
Mov = cat(3,  ctraj(T1,T2,n));

o_gripper = [ 0 -0.9408 -0.9408 -0.9408 -0.9408 ...
              0 0 0 -0.9408 -0.9408 -0.9408 -0.9408 0 0];

%for i = 0:3
%    Tarr(:,:,i+1)= Robot.fkine([0.8283-i*pi/6, 1.5238, 0.5880, -0.7])*transl(-3,0,0);
%end

for i = 0:50
 movePX(msg,cliente,[0.8283-i*pi/100, 1.5238, 0.5880, -0.7;0.8283-i*pi/100, 1.5238, 0.5880, -0.7], false);
end
Tarr(:,:,end+1)= Tarr(:,:,end)*transl(-5,0,0)
Tarr(:,:,end+1)= Robot.fkine([-1.3039,   0.9971,    2.3726,   -1.8203])

%for i = 1:7
%    Tarr(:,:,end+i) = Robot.fkine([-1.3039+i*pi/12,   0.9971,    2.3726,   -1.8203])
%end

%Mov = cat(3,  ctraj(T0,T1,n));
%view(70,20)


for i =1:length(Tarr)-1
Mov = cat(3,  ctraj(Tarr(:,:,i),Tarr(:,:,i+1),n));
for i=1:1*n
   thetas = InverseKinematics(Robot,l,Mov(:,:,i));
   movePX(msg,cliente,thetas, false);
   %Robot.plot(thetas(2,:),'notiles','noname')
   %hold on;
   %trplot(eye(4),'rgb','arrow','length',25,'frame','or')
   %hold on
   %plot3(Mov(1,4,i),Mov(2,4,i),Mov(3,4,i),'ro')
   %hold on;

%    if mod(i,n) == 0
%        movePX(msg,cliente,o_gripper(i/n), true);
%        pause(1);
%    end
end
end
%circ = [[0.8283, 1.5238, 0.5880, -0.5471, -1.3959],
 %       [] ]

for i = 1:75
 movePX(msg,cliente,[-1.3039+i*pi/100,   0.9971,    2.3726,   -1.8203;-1.3039+i*pi/100,   0.9971,    2.3726,   -1.8203], false);
end
a1=transl(5,0,0)*a ;
a2=transl(-5,5,0)*a1 ;
a3=transl(0,-5,0)*a2 ;

Mov = cat(3,  ctraj(a,a1,n),ctraj(a1,a2,n),ctraj(a2,a3,n));
for i=1:n*3
   thetas = InverseKinematics(Robot,l,Mov(:,:,i));
   movePX(msg,cliente,thetas, false);
   Robot.plot(thetas(2,:),'notiles','noname')
   hold on;
   trplot(eye(4),'rgb','arrow','length',25,'frame','or')
   hold on
   plot3(Mov(1,4,i),Mov(2,4,i),Mov(3,4,i),'ro')
   hold on;

%    if mod(i,n) == 0
%        movePX(msg,cliente,o_gripper(i/n), true);
%        pause(1);
%    end
end

expo=marcador(msg,Sub,cliente,Robot,l);