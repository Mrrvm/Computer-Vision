
function [points,obj] = find_objects(points,n)

    points = points(randperm(length(points),n),:);
    obj = zeros(length(points),1);

    current_obj = 1;
    index = 1;
    obj(index) = current_obj;

    while sum(obj==0)
        a = points(obj==0,:);
        [index,d] = knnsearch(points(obj==0,:),points(index,:));
        index = find(ismember(points,a(index,:),'rows'));
        if d>0.3
            current_obj = current_obj + 1;
        end

        obj(index) = current_obj;
    end

end