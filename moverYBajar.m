function MovAbajo = moverYBajar(inicial_arriba,zabajo,sub,Robot)
    n = 40;
    %debe comenzar la herramienta arriba en algun lugar
    a=sub.LatestMessage.Position;
    a = a(1:4);
    actual=Robot.fkine(a);
    trajInicial_arriba = ctraj(actual,inicial_arriba,n);
    trajBajar = ctraj(inicial_arriba,transl(0,0,zabajo)*inicial_arriba,n);
    MovAbajo = cat(3,trajInicial_arriba,trajBajar);
end