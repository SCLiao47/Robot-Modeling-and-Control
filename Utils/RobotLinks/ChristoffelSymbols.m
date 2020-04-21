function gamma = ChristoffelSymbols(i,j,k,M,th)

    gamma = 1/2*(diff(M(i,j),th(k))+diff(M(i,k),th(j))-diff(M(k,j),th(i)));
    
end
