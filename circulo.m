function circulo(msg,sub,cliente,Robot,l,expo)
if (expo==1)
a=sub.LatestMessage.Position;
a(5)=[];
a=Robot.fkine(a);
a2=Robot.fkine([0.0358,1.1965,1.5186,-1.1403]);%cambiar por posicion sobre el tablero
a1=transl(0,0,10)*a2;
% a = a1;
% a0=a;
% a0(3,4)=a1(3,4);
n=20;
Mov = cat(3,  ctraj(a,a1,n),ctraj(a1,a2,n));

    r = 3;
    step = 0.1;
    y1=-r:step:r;
    y2 = r:-step:-r;
    y = [y1,y2];
    x1 = sqrt(r^2-y1.^2);
    x2 = -x1;
    x = [x1, x2];
    nf = 20;
    n =40;
   
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
   
    %subir = transl(0,0,-zabajo)*current;
    trajSubir = ctraj(current,a1,n)
    Mov = cat(3,Mov,trajSubir);
    move(Mov,Robot,l,msg,cliente)
   
    %PLOTEAR----------------------------------------
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
