function punto(msg,sub,cliente,Robot,l,expo)
if (expo==1)
    a=sub.LatestMessage.Position;
    a(5)=[];
    a=Robot.fkine(a);
    n=20;

    a2=Robot.fkine([-0.3426,1.1249,1.9379,-1.4828]);%cambiar por posicion sobre el tablero sin incluir el transl
   
    a1=transl(0,0,10)*a2;
    arribaInicial = a1;
    Mov = cat(3,  ctraj(a,a1,n),ctraj(a1,a2,n));

    a = a2;
    c = 0.1;
    a1=transl(0,0,0)*a;
    a2=transl(c,0,0)*a1;
    Mov =cat(3, Mov, ctraj(a,a1,n),ctraj(a1,a2,n));
   
    for i = 1:4
    a11 =transl(0,0,5)*Mov(:,:,end);
    a12 =transl(0,3,0)*a11;
    a13 =transl(0,0,-5)*a12;
    a14 =transl(0,c,0)*a13;
    Mov = cat(3,Mov,ctraj( Mov(:,:,end),a11,n),ctraj(a11,a12,n),ctraj(a12,a13,n),ctraj(a13,a14,3));
    end
    Mov = cat(3,Mov,ctraj(Mov(:,:,end),transl(0,0,10)*Mov(:,:,end),n),ctraj(transl(0,0,10)*Mov(:,:,end) ,arribaInicial,n));
   
    move(Mov,Robot,l,msg,cliente);

else
    disp("El sistema no tiene marcador")
   
end