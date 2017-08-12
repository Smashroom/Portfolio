function [ fitness ] = Fitness(chromosome,prefer,CourseCapacity ) 
    fitness = 0;
    notlegal_flag = 0;
    
    for i = 1 : length(CourseCapacity)
        SelectedCourses = CourseConverter( prefer ,chromosome);
        n =sum ( SelectedCourses == i );
        if(CourseCapacity(i) >= n)
            continue
        else
            fitness = -2000;
            return;
        end
    end
    if(notlegal_flag == 0)
        fitness = (-1)*(sum(chromosome) - size(chromosome,1));
    end
end

