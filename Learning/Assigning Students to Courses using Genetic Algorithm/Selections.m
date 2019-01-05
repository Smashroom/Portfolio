function [ bestOut,bestOutInd ] = Selections( tmsel ,pop , prefer , courseCap , selectMethodNo  )

%% Tournament Selection

    if selectMethodNo ==1

        bestOutInd = floor(rand()*size(pop,2))+1;
        bestOut = pop{bestOutInd};

        for i = 1: tmsel
            newChildIndex = floor(rand()*size(pop,2))+1;
            newChild = pop{newChildIndex};
            fitBest = Fitness(bestOut,prefer,courseCap);
            fitInd = Fitness(newChild,prefer,courseCap);

            if(fitInd > fitBest)
                bestOut = newChild;
                bestOutInd = newChildIndex;
            end
        end
        
 %%Roulette Wheel       
    elseif selectMethodNo == 2
        sumFitness = 0;
        for i = 1: size(pop,2)
            sumFitness = sumFitness + Fitness(pop{i},prefer,courseCap);
        end
        
        for i = 1: size(pop,2)
            probArr(i) = sumFitness - Fitness(pop{i},prefer,courseCap);;
        end
        
        probArr =   probArr ./ (sumFitness*(length(pop)-1) );

        bestOutInd = randsrc(1,1,[1:size(pop,2);probArr]);
        bestOut = pop{bestOutInd};
    end

end

