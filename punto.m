function punto(msg,sub,cliente,Robot,l, expo)
    %%cinco puntos equidistantes
    if(expo == 1)
        movePX(msg,cliente,0, true);
        a=sub.LatestMessage.Position;
        a(5)=[];
        a=Robot.fkine(a);
        a1=transl(0,0,0)*a;
        a2=transl(0,0,5)*a1;
        a3=transl(5,0,5)*a2; 
        a4=transl(5,0,0)*a3; 
        a5=transl(5,0,5)*a4;
        a6=transl(10,0,5)*a5;
        a7=transl(10,0,0)*a6;
        a8=transl(10,0,5)*a7;
        a9=transl(15,0,5)*a8;
        a10=transl(15,0,0)*a9;
        a11=transl(15,0,5)*a10;
        a12=transl(20,0,5)*a11;
        a13=transl(20,0,0)*a12;
        a14=transl(20,0,5)*a13;

        Mov = cat(3,  ctraj(a,a1,n),ctraj(a1,a2,n),ctraj(a2,a3,n),ctraj(a3,a4,n), ctraj(a4,a5,n), ctraj(a5,a6,n), ctraj(a6,a7,n), ctraj(a7,a8,n), ctraj(a8,a9,n),  ctraj(a9,a10,n),  ctraj(a10,a11,n),  ctraj(a11,a12,n),  ctraj(a12,a13,n),  ctraj(a13,a14,n));

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