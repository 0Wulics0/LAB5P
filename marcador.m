function expo=marcador(msg,sub,cliente,Robot,l)

movePX(msg,cliente,0, true);
a=sub.LatestMessage.Position;
a(5)=[];
a=Robot.fkine(a);
a2=Robot.fkine([-0.4755,1.1863,1.3652,-1.0584]);
a1=transl(0,0,10)*a2;
a0=a;
a0(3,4)=a1(3,4);

n=20;

Mov = cat(3,  ctraj(a,a0,n),ctraj(a0,a1,n),ctraj(a1,a2,n));
for i=1:1*n*3
   thetas = InverseKinematics(Robot,l,Mov(:,:,i));
   movePX(msg,cliente,thetas, false);
%    Robot.plot(thetas(2,:),'notiles','noname')
%    hold on;
%    trplot(eye(4),'rgb','arrow','length',25,'frame','or')
%    hold on
%    plot3(Mov(1,4,i),Mov(2,4,i),Mov(3,4,i),'ro')
%    hold on;
end
movePX(msg,cliente,-1.42, true);
Mov = cat(3,  ctraj(a2,a1,n));
for i=1:1*n
   thetas = InverseKinematics(Robot,l,Mov(:,:,i));
   movePX(msg,cliente,thetas, false);
%    Robot.plot(thetas(2,:),'notiles','noname')
%    hold on;
%    trplot(eye(4),'rgb','arrow','length',25,'frame','or')
%    hold on
%    plot3(Mov(1,4,i),Mov(2,4,i),Mov(3,4,i),'ro')
%    hold on;
end
expo=1;
end