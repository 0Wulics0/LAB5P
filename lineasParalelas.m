function lineasParalelas(msg,sub,cliente,Robot,l, expo,z)
    if(expo == 1)
        a=sub.LatestMessage.Position;
        a(5)=[];
        a=Robot.fkine(a);
        %%tres lineas paralelas
        a00=Robot.fkine([0.3324,1.2323,1.3755,-1.0022]);
%         a00(3,4)=z;
        a0=transl(0,0,2)*a00;
        a1=transl(2,0,0)*a0;
        a2=transl(0,0,3)*a1;
        a3=transl(0,1,0)*a2; 
        a4=transl(0,0,-3)*a3; 
        a5=transl(-2,0,0)*a4;
        a6=transl(0,0,3)*a5;
        a7=transl(0,1,0)*a6;
        a8=transl(0,0,-3)*a7;
        a9=transl(2,0,0)*a8;
        a10=transl(0,0,3)*a9;
        
        n=20;
        n0=15;
        
        MovApr = cat(3, ctraj(a, a0, n0));
        
        for i=1:n0*1
           thetas = InverseKinematics(Robot,l,MovApr(:,:,i));
           movePX(msg,cliente,thetas, false);
           Robot.plot(thetas(2,:),'notiles','noname')
           hold on;
           trplot(eye(4),'rgb','arrow','length',25,'frame','or')
           hold on
           plot3(MovApr(1,4,i),MovApr(2,4,i),MovApr(3,4,i),'ro')
           hold on;

        %    if mod(i,n) == 0
        %        movePX(msg,cliente,o_gripper(i/n), true);
        %        pause(1);
        %    end
        end
        
        Mov = cat(3,  ctraj(a0,a00,n),ctraj(a00,a1,n),ctraj(a1,a2,n),ctraj(a2,a3,n),ctraj(a3,a4,n), ctraj(a4,a5,n), ctraj(a5,a6,n), ctraj(a6,a7,n), ctraj(a7,a8,n), ctraj(a8,a9,n), ctraj(a9,a10, n),ctraj(a10,a,n) );
        
        for i=1:n*12
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

