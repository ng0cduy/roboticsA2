function boxes = SetGoodsPick
    goodsX = -0.8;
    goodsY = -0.5;
    goodsBaseZ = 0;
    goodsBaseZ = goodsBaseZ+0.06; % for the goods's pose position at the center of the goods
                                 % z of goods = 0.12;
    
    boxes = cell(3,1);
    for i = 1:3
        choice = randi(5);
        hold on
        switch choice
            case 1
                boxes{i} = goods('green.ply',transl(goodsX,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi)); %z of goods = 0.12; 
            case {2,3} 
                boxes{i} = goods('red.ply',transl(goodsX,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi));
            otherwise
                boxes{i} = goods('blue.ply',transl(goodsX,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi));
        end
    end
end