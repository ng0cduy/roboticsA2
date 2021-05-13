function boxes = SetGoodsPick
%     choice = {};
    goodsX = -0.8;
    goodsY = -0.5;
    goodsBaseZ = 0;
    goodsBaseZ = goodsBaseZ+0.06; % for the goods's pose position at the center of the goods
                                 % z of goods = 0.12;
    n = 6;
    choice = randperm(n,n);
    boxes = cell(6,1);
    for i  = 1:n
        switch choice(i)
            case {1,2}
                ply_name = 'green.ply';
            case {3,4}
                ply_name = 'red.ply';
            case {5,6}
                ply_name = 'blue.ply';
        end
        if i <=3
            boxes{i} = goods(ply_name,transl(goodsX,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi));
        else
            boxes{i} = goods(ply_name,transl(-0.2,goodsY,goodsBaseZ+0.12*(i-4))*troty(pi));
        end
            
    end 
%     for i = 1:n
% %         choice = randi(5);
%         hold on
%         switch choice(i)
%             case 1
%                 boxes{i} = goods('green.ply',transl(goodsX,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi)); %stacked on the previous goods 
%             case 2 
%                 boxes{i} = goods('red.ply',transl(goodsX,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi));
%             case 3
%                 boxes{i} = goods('blue.ply',transl(goodsX,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi));
%         end
%     end
%     n = 3;
%     choice = randperm(n,n);
%     boxes = cell(3,1);
%     for i = 1:n
% %         choice = randi(5);
%         hold on
%         switch choice(i)
%             case 1
%                 boxes{i} = goods('green.ply',transl(-0.2,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi)); %stacked on the previous goods 
%             case 2 
%                 boxes{i} = goods('red.ply',transl(-0.2,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi));
%             case 3
%                 boxes{i} = goods('blue.ply',transl(-0.2,goodsY,goodsBaseZ+0.12*(i-1))*troty(pi));
%         end
%     end
    
end