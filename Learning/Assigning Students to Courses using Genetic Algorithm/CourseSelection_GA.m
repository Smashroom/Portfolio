clear
clc 
close all

iterationSize = 100;
popSize = 10;

small = 20;
big = 30 ;

crowd = 'SMALL';
dist = 'NONUNIFORM' ;

crossRate = 0.5;
MutationRate = 0.5;

tmsel = 10;
courseCap = [8 8 8 8];
popi = [10 5 5 2];

for l = 1 : 10
    
    if(strcmp(crowd,'SMALL'))
        prefer = zeros(small,4);
    else 
        prefer = zeros(big,4);
    end

    distArr = [];
    
    if(strcmp(dist,'UNIFORM'))
        for i = 1:size(prefer,1)
            prefer(i,:) = randperm(4);
        end
    else
        for i = 1:size(prefer,1)
            courses = [1 2 3 4];
            for j = 1 : size(prefer,2)
                distArr = [];
                for k = 1: size(courses,2)
                    distArr = [distArr repmat(courses(k),1,popi(courses(k)))];
                end

                idx = floor(size(distArr,2)*rand())+1;
                prefer(i,j) = distArr(idx);
                courses = setdiff(courses,distArr(idx));
            end
        end

    end

    pop = PopulationInit(prefer,popSize,courseCap);

    nextPop = pop;
    
    childBest = AnalyzePopu(nextPop,prefer,courseCap);
    
    popBest(l) = Fitness(childBest,prefer,courseCap);
    
    
    for i = 1 : iterationSize
        
       
        % Elitism
        best = AnalyzePopu(nextPop,prefer,courseCap);
        best_index = FindInd( best,nextPop );

        
        pop(best_index) = [];
        
        % Mutation 
        for j = 1 :  length(pop)
            if(MutationRate < rand())
                 continue;
             end
             mutation_idx = randsample(length(pop{j}),floor(rand()*length(pop{j}))+1);
             chromosome = pop{j};
             chromosome(mutation_idx) = floor(rand()*4)+1;
             nextPop{j} = chromosome;
        end
        
        % Crossing Over
        for m = 1 : length(nextPop)
            if (rand() > crossRate )
               continue;
            end
            divIndex = floor(rand()*length(nextPop{1})) + 1;

            [p1,~]= Selections(tmsel,nextPop,prefer,courseCap,1);
            [p2,~]= Selections(tmsel,nextPop,prefer,courseCap,1);

            temp = p1(1:divIndex);
            p1(1:divIndex) = p2(1:divIndex);
            p2(1:divIndex) = temp;

            nextPop{end+1} = p1;
            nextPop{end+1} = p2;
        end
        
        % Next Generation is chosen
        for k = 1 : length(pop)-1
            [pop{k},~] = Selections( tmsel,nextPop,prefer,courseCap,2);
        end
        
        pop = insert(pop,best,best_index);
        theStrongest = AnalyzePopu(pop,prefer,courseCap);
        nextPop = pop;

        strongestFitness = Fitness(theStrongest,prefer,[8 8 8 8]);
     
        if(strongestFitness == 0) 
            break;
        end
    end
    
    strongs{l} = theStrongest;
    SelectedCourses{l} =CourseConverter( prefer ,theStrongest);

    strongLeague(l) = strongestFitness;

end
figure
subplot(1,2,1)
plot(popBest,'r')
legend('Fitness vs Preference Before the Selections')
xlabel('Preference No')
ylabel('Fitness')
subplot(1,2,2)
plot(strongLeague,'g')
legend('Fitness vs Preference After the Selections')
xlabel('Preference No')
ylabel('Fitness')
