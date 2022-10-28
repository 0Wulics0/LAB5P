function move(Mov,Robot,l,msg,cliente)
    sz = size(Mov)
   
    for i=1:sz(3)
       thetas = InverseKinematics(Robot,l,Mov(:,:,i));
       movePX(msg,cliente,thetas, false);
    end
   
end