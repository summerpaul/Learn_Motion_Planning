% A星算法的输入:有起点与终点的障碍物地图
% 输出是最短路径
% 图搜索需要关注:维护一个OPEN list 表示待检查的点,维护一个CLOSED List, 表示无需访问的节点
function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    
    % 地图障碍物的数量+起点+终点
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    % 
    % MAP中所有的值,赋值为2
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    % 获取目标点
    % 目标点是最后的点
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    % 目标点在MAP中的值为0
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    % 有障碍物的点赋值为-1
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    % 起始点为1,起始点为地图中第一个数据
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    % 维护一个拓展即将访问所有节点的容器
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    % 将所有的障碍物放入到CLOSED队列中,表示不能被访问的点
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    %  计算当前点到目标点的距离(启发函数的值)
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    % 优先队列中,放入初始节点
    % OPEN 队列表示代检查的点
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    %  访问第一个点(源点到源点)    
    OPEN(OPEN_COUNT,1)=0;
    % 记录被访问过的节点     
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 跳出循环的条件:没有需要被访问的节点
%     [r, c] = size(OPEN(OPEN_COUNT, :));
    while(length(OPEN(OPEN_COUNT,:)) ~= 0) %you have to dicide the Conditions for while loop exit 
        % 弹出优先队列中的最小的元素,并放入到CLOSE列表中
        node_x = OPEN(OPEN_COUNT,2);
        node_y = OPEN(OPEN_COUNT,3);
        
        % 判断最后搜索的点是否被扩展
        if(node_x == xTarget && node_y == yTarget)
            break;
        end
%         gn = path_cost;
        % 扩展新的节点
        i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget)
        % 寻找当前点待扩展的点
        exp_array = expand_array(node_x,node_y,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y)
        % 找到fn最小点的x y
        fn = exp_array(:,5);
        i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget)
%         i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget)

        min_index = find(fn == min(fn))
        if length(min_index) > 1
            min_index = min_index(2)
        end
        xNode_new = exp_array(min_index, 1)
        yNode_new = exp_array(min_index, 2)
        
        goal_distance=distance(xNode_new,yNode_new,xTarget,yTarget);
        path_cost=path_cost + exp_array(min_index, 3);
        fn_new = path_cost + goal_distance;
        % 更新OPENlist
        OPEN_COUNT = OPEN_COUNT +1;
        OPEN(OPEN_COUNT,:)=insert_open(xNode_new,yNode_new,xNode,yNode,goal_distance,path_cost,fn_new);
        CLOSED_COUNT=CLOSED_COUNT+1;
        
        
        CLOSED(CLOSED_COUNT,1)=xNode_new;
        CLOSED(CLOSED_COUNT,2)=yNode_new;
        xNode = xNode_new;
        yNode = xNode_new;
        
        
%         n_index = node_index(OPEN,exp_array(min_index,1),exp_array(min_index,2)) 
        
%         i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
        
        
        
%         [r, c] = size(OPEN);
%         [r, c] = size(OPEN);
        
        
        
        
     %
     %finish the while loop
     %
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
    
   path = [];
   [m, n] = size(OPEN)
   for i=1:m
       path(i, 1)=  OPEN(i, 2);
       path(i, 2) = OPEN(i, 3);
   
   
   
end
