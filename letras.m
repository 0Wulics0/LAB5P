function letras(msg,sub,cliente,Robot,l, expo)

if(expo == 1)
    a=sub.LatestMessage.Position;
    a(5)=[];
    a=Robot.fkine(a);
    %a
    a00=Robot.fkine([-0.6545,1.1403,1.6874,-1.2118]);
%   a00(3,4)=z;
    a0=transl(0,0,2)*a00;
    a2=transl(0,2,0)*a00;
    a3=transl(2,0,0)*a2; 
    a4=transl(0,-2,0)*a3; 
    a5=transl(0,0,2)*a4;
    a6=transl(-2,1,0)*a5;
    a7=transl(0,0,-2)*a6;
    a8=transl(2,0,0)*a7;
    a9=transl(0,0,2)*a8;
    %m
    a10=transl(1,-1,0)*a9;
    a11=transl(0,0,-2)*a10;
    a12=transl(0,2,0)*a11;
    a13=transl(1,-1,0)*a12;
    a14=transl(1,1,0)*a13;
    a15=transl(0,-2,0)*a14;
    a16=transl(0,0,2)*a15;

    n = 20;
    n0 = 15;
    
    Mov = cat(3,  ctraj(a,a0,n0));
    move(Mov,Robot,l,msg,cliente);
    
    MovAcerca = cat(3, ctraj(a0,a00,n));
    move(MovAcerca,Robot,l,msg,cliente);
    
    MovLetraA = cat(3, ctraj(a00, a2, n0), ctraj(a2,a3,n0),ctraj(a3,a4,n0), ctraj(a4,a5,n0), ctraj(a5,a6,n0)); 
    move(MovLetraA,Robot,l,msg,cliente);
    
    MovAcercamientoA = cat(3, ctraj(a6,a7,n));
    move(MovAcercamientoA,Robot,l,msg,cliente);
    
    MovFinalA = cat(3,ctraj(a7,a8,n0),ctraj(a8,a9,n0),ctraj(a9,a10,n0));
    move(MovFinalA,Robot,l,msg,cliente);
    
    MoveAcercamientoM = cat(3, ctraj(a10,a11,n));
    move(MoveAcercamientoM,Robot,l,msg,cliente);
    
    MoveM = cat(3, ctraj(a11,a12,n0), ctraj(a12,a13,n0), ctraj(a13,a14,n0), ctraj(a14, a15, n0),ctraj(a15,a16,n0));
    move(MoveM,Robot,l,msg,cliente);

else
    Alerta = 'El sistema no tiene marcador';
    disp(Alerta)
end
end

