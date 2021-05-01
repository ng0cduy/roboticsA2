        % Get destination of the goods        
        function goodsDes = GetGoodsDes(obj,color,order)            
        
            if strcmpi(color,'red') == 1
                type = 1;
            
            elseif strcmpi(color,'blue') == 1
                type = 2;
              
            elseif strcmpi(color,'green') == 1
                type = 3;
            else
                type = -1;
                msg = 'this one''s color is not for recognition';
                error(msg);
            end
            
            % Dimension of a goods
            [x,~,z] = obj.getGoodsSize;
        
            % Set goods destinations coordinate matrix from the lowest to
            % the highest, categorized by color 
            % we intend to move 9 goods in total, 3 of each
            dropMatrix = zeros(3,3,3);
            
            dropMatrix(1,:,1) = [0.3, -0.3, z/2]; % first red 
            dropMatrix(1,:,2) = [0.3 + x, -0.3, z/2]; % first blue 
            dropMatrix(1,:,3) = [0.3 + 2*x, -0.3, z/2]; % first red 
    
            for i = 2: 3
                dropMatrix(i,:,1) = [dropMatrix(1,1:2,1), dropMatrix(i-1,3,1)+z];
                dropMatrix(i,:,2) = [dropMatrix(1,1:2,2), dropMatrix(i-1,3,2)+z];
                dropMatrix(i,:,3) = [dropMatrix(1,1:2,3), dropMatrix(i-1,3,3)+z];
            end
            goodsDes = dropMatrix(order,:,type);
        end
