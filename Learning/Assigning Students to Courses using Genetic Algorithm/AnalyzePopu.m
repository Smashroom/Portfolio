function [ best ] = AnalyzePopu( pop,prefer ,courseCap)

    bestFitness = -5555;
    
    for i =  1 : length(pop)
        
        childFitness = Fitness(pop{i},prefer,courseCap);
        if bestFitness < childFitness
            bestFitness = childFitness;
            best = pop{i};
        end
    end
    

end

