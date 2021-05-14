%% function to spawn 6 boxes in random colors for 2 stacks 
function boxes = SetGoodsPick
    % Position to spawn the 1st box in the first stack
    goodsX = -0.8;
    goodsY = -0.5;
    goodsBaseZ = 0;
    goodsBaseZ = goodsBaseZ+0.06; % the goods's pose position at the center of the goods
                                 % z of goods = 0.12;
    n = 6;
    choice = randperm(n,n);       % create array for boxes' colors
    boxes = cell(6,1);            % create a cell array for 6 boxes
    for i  = 1:n
        % each number in choice array corresponds with a color
        switch choice(i)
            case {1,2}
                ply_name = 'green.ply';
            case {3,4}
                ply_name = 'red.ply';
            case {5,6}
                ply_name = 'blue.ply';
        end
        % spawn goods
        if i <=3
            boxes{i} = goods(ply_name,transl(goodsX,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi));
        else
            boxes{i} = goods(ply_name,transl(-0.2,goodsY,goodsBaseZ+0.12*(i-4))*troty(pi));
        end
            
    end 
end