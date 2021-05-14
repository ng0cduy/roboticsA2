%% Get destination transform of the goods        
function goodsDesTr = GetGoodsDes(obj,order)            
    
    if strcmpi(obj.color,'red') == 1
        type = 1;

    elseif strcmpi(obj.color,'blue') == 1
        type = 2;

    elseif strcmpi(obj.color,'green') == 1
        type = 3;
    else
        type = -1;
        msg = 'Cannot recognise the color or this one''s color is not for recognition';
        error(msg);
    end

    % Dimension of a goods
    [~,y,z] = obj.getGoodsSize;

    % Set goods destinations coordinate matrix from the lowest to
    % the highest, categorized by color 
    % 3 is the maximum number of one type of goods that we can move
    dropMatrix = zeros(3,3,3);

    dropMatrix(1,:,1) = [1.25, 0.2, z];     % first red 
    dropMatrix(1,:,2) = [1.25, 0.2+y, z];   % first blue 
    dropMatrix(1,:,3) = [1.25, 0.2+2*y, z]; % first green 

    for i = 2: 3
        dropMatrix(i,:,1) = [dropMatrix(1,1:2,1), dropMatrix(i-1,3,1)+z];
        dropMatrix(i,:,2) = [dropMatrix(1,1:2,2), dropMatrix(i-1,3,2)+z];
        dropMatrix(i,:,3) = [dropMatrix(1,1:2,3), dropMatrix(i-1,3,3)+z];
    end

    % extract the demand position
    goodsDes = dropMatrix(order,:,type);
    goodsDesTr = transl(goodsDes(1), goodsDes(2), goodsDes(3))*troty(pi);
end
