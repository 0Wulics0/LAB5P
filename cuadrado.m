function cuadrado(msg,sub,cliente,Robot,l, expo)

if(expo == 1)
    movePX(msg,cliente,0, true);
    a=sub.LatestMessage.Position;
    a(5)=[];
    a=Robot.fkine(a);
    a1=transl(5,0,0)*a;
    a2=transl(5,5,0)*a1;
    a3=transl(0,5,0)*a2; 
    a4=transl(0,0,0)*a3;

    Mov = cat(3,  ctraj(a,a1,n),ctraj(a1,a2,n),ctraj(a2,a3,n),ctraj(a3,a4,n));

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
else
    Alerta = 'El sistema no tiene marcador';
    disp(Alerta)
end
end