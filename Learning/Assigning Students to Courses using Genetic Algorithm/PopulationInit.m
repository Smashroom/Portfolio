function [ pop ] = PopulationInit( prefer,sizeOfPop,courseCap )
    
    for i = 1 : sizeOfPop
        j = 0;
        CourseCap = courseCap; 
        while ( j < size(prefer,1) )

            selectedCourse = randi(4);
            % If course storage is full , continue for other courses
            if CourseCap(selectedCourse) == 0
                continue

            % If course storage is okey , assign the course and decrease the
            % storage of the course
            else
                j = j + 1;
                assignedCourses(j) = selectedCourse;
                CourseCap(selectedCourse) = CourseCap(selectedCourse) - 1;
            end
        end
        for k = 1 : size(prefer,1)
            chromosome(k) = prefer(k,assignedCourses(k));
        end
        pop{i} = chromosome';
    end
end

