function dibujoLibre(msg,sub,cliente,Robot,l,expo)
if (expo==1)
a=sub.LatestMessage.Position;
a(5)=[];
a=Robot.fkine(a);
a2=Robot.fkine([0.7159,1.3141,1.2323,-1.0431]);%cambiar por posicion sobre el tablero
a1=transl(0,0,10)*a2;
% a = a1;
% a0=a;
% a0(3,4)=a1(3,4);
n=20;
Mov = cat(3,  ctraj(a,a1,n),ctraj(a1,a2,n))

%HACER LINEAS RECTA DEL INICIO
    I0 =     Mov(:,:,end);
    I1 = transl(1,-1,0)*Mov(:,:,end);
    Mov = cat(3,Mov,ctraj(I0,I1,n));


    r = 2;
    step = 0.1
    y1=-r:step:r*0.1
    th = -pi/2:0.1:pi/2
    y = r.*sin(th)
    %y = [y1]
    x = r.*cos(th)%sqrt(r^2-y1.^2)
    n =20
   
    current=Mov(:,:,end);
   
    %Hacer el mov del circulo
    for i = 1:length(x)-1
        transx(i) = x(i) - x(i+1);
        transy(i) = y(i) - y(i+1);
    end
    %Mov = a2
    for i = 1:length(x)-1
        next = transl(transx(i),transy(i),0)*current;
        Mov = cat(3,Mov,next);
        current = next;
    end
   
   
    I0 =     Mov(:,:,end);
    I1 = transl(-1,-1,0)*Mov(:,:,end);
    Mov = cat(3,Mov,ctraj(I0,I1,n));
   


    current = Mov(:,:,end);
    subir = transl(0,0,10)*current;
    trajSubir = ctraj(current,subir,n)
    Mov = cat(3,Mov,trajSubir,ctraj(subir, a,n));
   
    move(Mov,Robot,l,msg,cliente);
   
%     sz = size(Mov)
% for i=1:sz(3)
%    thetas = InverseKinematics(Robot,l,Mov(:,:,i));
%    %movePX(msg,cliente,thetas, false);
%    Robot.plot(thetas(2,:),'notiles','noname')
%    hold on;
%    trplot(eye(4),'rgb','arrow','length',25,'frame','or')
%    hold on
%    
%    plot3(Mov(1,4,i),Mov(2,4,i),Mov(3,4,i),'ro')
%    hold on;
% %    if mod(i,n) == 0
% %        movePX(msg,cliente,o_gripper(i/n), true);
% %        pause(1);
% %    end
% end
else
    disp("El sistema no tiene marcador")
end