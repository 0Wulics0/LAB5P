function [expo, z]=marcador(msg,sub,cliente,Robot,l)

movePX(msg,cliente,0, true);
a=sub.LatestMessage.Position;
a(5)=[];
a=Robot.fkine(a);
a2=Robot.fkine([-0.4755,1.1863,1.3652,-1.0584]);
a1=transl(0,0,10)*a2;
a0=a;
a0(3,4)=a1(3,4);

z=a1(3,4);

n=20;

Mov = cat(3,  ctraj(a,a0,n),ctraj(a0,a1,n),ctraj(a1,a2,n));
move(Mov,Robot,l,msg,cliente);
movePX(msg,cliente,-1.42, true);

Mov = cat(3,  ctraj(a2,a1,n));
move(Mov,Robot,l,msg,cliente);
movePX(msg,cliente,-1.72, true);

expo=1;
end