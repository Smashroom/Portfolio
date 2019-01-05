function [ SelectedCourses ] = CourseConverter( prefer ,chromosome)
    for k = 1 : size(prefer,1)
        SelectedCourses(k) = prefer(k,chromosome(k));
    end
end

