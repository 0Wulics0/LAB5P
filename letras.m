function letras(msg,sub,cliente,Robot,l, expo)

if(expo == 1)
    a=sub.LatestMessage.Position;
    %a
    a2=Robot.fkine([-0.4755,1.1863,1.3652,-1.0584]);%cambiar por posicion sobre el tablero
   
a1=transl(0,0,10)*a2;
Mov = cat(3,  ctraj(a,a1,n),ctraj(a1,a2,n));

%     a = a2;
%     a1=transl(0,0,0)*a;
%     a2=transl(0,10,0)*a1;
%     a3=transl(5,10,0)*a2;
%     a4=transl(5,0,0)*a3;
%     a5=transl(5,0,5)*a4;
%     a6=transl(0,5,5)*a5;
%     a7=transl(0,5,0)*a6;
%     a8=transl(5,5,0)*a7;
%     a9=transl(5,5,5)*a8;
   
    a = a2;
    c = 3;
    a1=transl(0,0,0)*a;
    a2=transl(0,2*c,0)*a1;
    a3=transl(c,0,0)*a2;
    a4=transl(0,-2*c,0)*a3;
    a5=transl(0,0,c)*a4;
    a6=transl(-c,c,0)*a5;
    a7=transl(0,0,-c)*a6;
    a8=transl(c,0,0)*a7;
    %a9=transl(5,5,5)*a8;
   
    a9 =  transl(0,0,c)*a8
    a10 = transl(c,-c,0)*a9
    a11 =transl(0,0,-c)*a10
    a12 = transl(0,2*c,0)*a11
    a13 = transl(c,0,0)*a12
    a14 = transl(0,-2*c,0)*a13
    a15 = transl(0,0,c)*a14
    a16 = transl(0,2*c,0)*a15
    a17 = transl(0,0,-c)*a16
    a18 = transl(c,0,0)*a17
    a19 = transl(0,-2*c,0)*a18
    a20 = transl(0,0,10)*a19
   
   
   
   
   
   
    %m
%     a10=transl(10,0,5)*a9;
%     a11=transl(10,0,0)*a10;
%     a12=transl(10,10,0)*a11;
%     a13=transl(13,7,0)*a12;
%     a14=transl(16,10,0)*a13;
%     a15=transl(16,0,0)*a14;

    Mov2 = cat(3,  ctraj(a,a1,n),ctraj(a1,a2,n),ctraj(a2,a3,n),ctraj(a3,a4,n),...
    ctraj(a4,a5,n), ctraj(a5,a6,n), ctraj(a6,a7,n), ctraj(a7,a8,n), ctraj(a8,a9,n),  ...
    ctraj(a9,a10,n),  ctraj(a10,a11,n),  ctraj(a11,a12,n),  ctraj(a12,a13,n),  ctraj(a13,a14,n), ctraj(a14, a15, n),...
    ctraj(a15,a16,n),  ctraj(a16,a17,n),  ctraj(a17,a18,n), ctraj(a18,a19,n), ctraj(a19,a20,n), ctraj(a20,a1,n));
Mov = cat(3,Mov,Mov2)
    sz = size(Mov)
    for i=1:sz(3)
       thetas = InverseKinematics(Robot,l,Mov(:,:,i));
       movePX(msg,cliente,thetas, false);
    end
else
end
end   

