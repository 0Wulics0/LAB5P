function move(Mov,Robot,l,msg,cliente)
     sz = size(Mov);
     figure(1)
     for i=1:sz(3)
     thetas = InverseKinematics(Robot,l,Mov(:,:,i));
     movePX(msg,cliente,thetas, false);
%    hold on;
%    trplot(eye(4),'rgb','arrow','length',25,'frame','or')
%    hold on;
%    plot3(Mov(1,4,i),Mov(2,4,i),Mov(3,4,i),'ro')
%    hold on;
%    xlim([-20,20])
%    ylim([-20,20])
%    Robot.plot(thetas(2,:),'notiles','noname')
     end
   
   
end