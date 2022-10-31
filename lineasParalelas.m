function lineasParalelas(msg,sub,cliente,Robot,l, expo)
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
        
        n0=10;
        MovApr = cat(3, ctraj(a, a0, n0));
        move(MovApr,Robot,l,msg,cliente);
        
        n=20;
        MovAcerca = cat(3,  ctraj(a0,a00,n));
        move(MovAcerca, Robot,l, msg, cliente);
        
        MovLinea1 = cat(3, ctraj(a00,a1,n0),ctraj(a1,a2,n0),ctraj(a2,a3,10));
        move(MovLinea1,Robot,l,msg,cliente);
        
        MoveAcercarZ2 = cat(3,ctraj(a3,a4,n));
        move(MoveAcercarZ2,Robot,l,msg,cliente);
        
        MovLinea2 = cat(3, ctraj(a4,a5,n0),ctraj(a5,a6,n0),ctraj(a6,a7,10));
        move(MovLinea2,Robot,l,msg,cliente);
        
        MoveAcercarZ3 = cat(3,ctraj(a7,a8,n));
        move(MoveAcercarZ3,Robot,l,msg,cliente);
        
        MovLinea3 = cat(3, ctraj(a8,a9,n0),ctraj(a9,a10,n0));
        move(MovLinea3,Robot,l,msg,cliente);        
    else
        Alerta = 'El sistema no tiene marcador';
        disp(Alerta)
    end
end

