function cost = cost_f(x,lambda)
    global optimParams
    %% Variable
    % x(1) --> a
    % x(2) --> b
    % x(3) --> x_1
    % x(4) --> y_1
    % x(5) --> x_2
    % x(6) --> y_2
    % x(7) --> x_3
    % x(8) --> y_3
    % x(9) --> x_4
    % x(10) --> y_4
    
    R = optimParams.radius;  
    objective = x(1)*x(2) - pi*(R(1)^2 + R(2)^2 + R(3)^2 + R(4)^2);
    %Constraint 1: 1-6 --> Constraint due to collision between circles
    %             7-22 --> Constraint due to exceeding the rectangular box
    %             side lenghts
    %             7-14 --> Relation between side length + position + radius
    %            14-22 --> Relation between position + radius
    
    %Constraint Array
    c = [R(1)+R(2)-sqrt((x(3)-x(5))^2 + (x(4)-x(6))^2); ...
         R(1)+R(3)-sqrt((x(3)-x(7))^2 + (x(4)-x(8))^2);...
         R(1)+R(4)-sqrt((x(3)-x(9))^2 + (x(4)-x(10))^2);...
         R(2)+R(3)-sqrt((x(5)-x(7))^2 + (x(6)-x(8))^2);...
    	 R(2)+R(4)-sqrt((x(5)-x(9))^2 + (x(6)-x(10))^2);...
    	 R(3)+R(4)-sqrt((x(7)-x(9))^2 + (x(8)-x(10))^2);...
         x(3)+R(1)-x(1); x(5)+R(2)-x(1); x(7)+R(3)-x(1);...
         x(9)+R(4)-x(1); x(4)+R(1)-x(2); x(6)+R(2)-x(2);...
         x(8)+R(3)-x(2); x(10)+R(4)-x(2);(R(1)-x(3)); ...
         (R(2)-x(5)); (R(3)-x(7)); (R(4)-x(9)); (R(1)-x(4)); ...
         (R(2)-x(6)); (R(3)-x(8)); (R(4)-x(10))];
    
    %Penalty Flag List
    pF = c > 0;
    penalty = 0;
    for i = 1:size(pF,1)
        penalty = penalty + pF(i)*(c(i)^2);  
    end
     
        
    cost =  objective + lambda*penalty;
end
