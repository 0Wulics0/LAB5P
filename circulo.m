function circulo(msg,sub,cliente,Robot,l,zabajo)
    r = 3;
    step = 0.01
    y1=-r:step:r
    y2 = r:-step:-r
    y = [y1,y2]
    x1 = sqrt(r.^2-y1.^2)
    x2 = -x1
    x = [x1, x2]
    nf = 20; ns =40; zabajo = -5;
    inicial_arriba = Robot.fkine([0.0818, 0.5062, 1.7129, -0.6085]);
    Mov = moverYBajar(inicial_arriba,zabajo,sub,Robot)
    current=Mov(:,:,end);
    for i = 1:length(x)
        
        next = transl(x(i),y(i),0)*current;
        traj=ctraj(current,next,20)
        Mov = cat(3,Mov,traj);
        
        current = next;
    end
    subir = transl(0,0,-zabajo)*current;
    trajSubir = ctraj(current,subir,nf)
    Mov = cat(3,Mov,trajSubir);
    move(Mov,Robot,l,msg,cliente)
end