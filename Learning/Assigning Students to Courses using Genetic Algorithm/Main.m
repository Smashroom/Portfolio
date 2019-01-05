clear all
clc
close all

%% Definitions
Balanced = [5,5,5,5];
Unbalanced = [10,5,5,2];
Low = 20;
High = 30;
course_capacity = [8,8,8,8];

%% Properties
Crowdness = Low;
Popularity = Balanced;
course_number = 4;
sample_space = 10;

Population_Size = 50;
Crossover_Rate = 0.85;
Mutation_Rate = 0.1;
Offspring_Population = 20;
Tournament_Size = 5;
termination = 500;


standard_matrix = 1:course_number;
standard_matrix = repmat(standard_matrix,Crowdness,1);
%% Preference Matrix Generator
Preference_Cell = cell(1,sample_space);

for q = 1:sample_space
    Preferences = zeros(Crowdness,course_number);
    for i = 1: Crowdness
        s = zeros(1,course_number);
        t = Popularity;
        for j = 1: course_number
            Distribution = t/sum(t);
            Distribution = cumsum(Distribution);
            r = rand;
            if r < Distribution(1)
                s(j)=1;
                t(1)=0;
            else if r < Distribution(2)
                    s(j)=2;
                    t(2)=0;
                else if r < Distribution(3)
                        s(j)=3;
                        t(3)=0;
                    else if r < Distribution(4)
                            s(j)=4;
                            t(4)=0;
                        end
                    end
                end
            end
        end
        Preferences(i,:)=s;
    end
    Preference_Matrix{q} = Preferences;
end

%% Initial Solution Generator

for prf = 1: sample_space
    Preference = Preference_Matrix{1,prf};
    
    
    
    Result = zeros(Crowdness,course_number);
    
    Population = zeros(Crowdness,Population_Size);
    for p = 1:Population_Size
        random_order = randperm(Crowdness);
        courses = course_capacity;
        
        for s = 1:Crowdness
            s_prime = random_order(s);
            pref = Preference(s_prime,:);
            res = zeros(1,course_number);
            
            tr = 1;
            while(tr)
                r = ceil(course_number*rand);
                if courses(r)>0
                    tr = 0;
                end
            end
            res(r)=pref(r);
            courses(r)=courses(r)-1;
            courses(r)=max(courses(r),0);
            Result(s_prime,:)=res;
        end
        Population(:,p)=sum(Result,2)';
        
    end
    Initial_Population = Population;
    %% Evolution
    
    result(1)=0;
    result_2(1)=0;
    m(1)=0;
    
    Population = Initial_Population;
    for turn = 1:termination
        %% Mutation
        random_order = randperm(Population_Size);
        for e = 1: Population_Size
            
            e_prime = random_order(e);
            Individual = Population(:,e_prime);
            
            for e_2 = 1:Crowdness
                r_mutate = rand;
                if r_mutate < Mutation_Rate
                    r_mutate_3 = randi(course_number);
                    Individual(e_2)=r_mutate_3;
                end
            end
            Population (:, e_prime) = Individual;
        end
        
        %% Parent Selection
        Offsprings = zeros(Crowdness,Offspring_Population);
        for p = 1:Offspring_Population/2
            
            random_order = randperm(Population_Size);
            Contestants = Population(:,random_order(1:Tournament_Size));
            Fitness_Contestants = sum(Contestants)-Crowdness;
            [~,best_places] = sort(Fitness_Contestants);
            Sorted_Contestants(:,best_places) = Contestants;
            
            Individual_1 = Sorted_Contestants(:,1);
            Individual_2 = Sorted_Contestants(:,2);
            
            
            %% Crossing Over
            
            r_cross = rand;
            if r_cross < Crossover_Rate
                
                cross_point = randi(Crowdness);
                offspring_1 = Individual_2;
                offspring_1(1:cross_point) = Individual_1(1:cross_point);
                offspring_2 = Individual_1;
                offspring_2(1:cross_point) = Individual_2(1:cross_point);
                
                Offsprings(:,2*p-1)= offspring_1;
                Offsprings(:,2*p) = offspring_2;
                
            else
                Offsprings(:,2*p-1)= Individual_1;
                Offsprings(:,2*p) = Individual_2;
            end
            
        end
        
        %% Join Generations
        Population = [Population,Offsprings];
        %% Evaluate Fitness
        Cost = sum(Population,1)-Crowdness;
        Fitness = Crowdness * (course_number) - Cost;
        for e = 1: size(Population,2)
            test_ind = Population(:,e);
            locations = Preference==test_ind;
            locations = sum (locations .* standard_matrix,2);
            occurence = histcounts(locations);
            if any(occurence > max(course_capacity))
                Fitness(e)=0;
            end
            
        end
        
        if any (Fitness == Crowdness * (course_number))
            terminated_turn = turn
            [~,best_place] = max (Fitness);
            best_solution = Population(:,best_place)
            best_cost = sum(best_solution)-Crowdness
            plot(Overall_Fitness)
            break
        end
        
        
        
        
        
        %% Elitism
        
        Cost = sum(Population,1)-Crowdness;
        [~,best_place] = min (Cost);
        Next_Generation(:,1) = Population(:,best_place);
        %Best_Cost(turn) = best_cost;
        %Average_Cost(turn) = mean(Cost);
        
        %% Roulette Wheel
        
        Roulette_Distribution = Fitness/sum(Fitness);
        Roulette_Distribution = cumsum(Roulette_Distribution);
        
        for s = 2: Population_Size
            r_roulette = rand;
            for t = 1: length(Roulette_Distribution)
                if r_roulette < Roulette_Distribution(t)
                    Next_Generation(:,s) = Population(:,t);
                    break
                end
            end
        end
        Population = Next_Generation;
    end
    
    terminated_turn = turn;
    Cost = sum(Population,1)-Crowdness;
    Fitness = Crowdness * (course_number) - Cost;
    [~,best_place] = max (Fitness);
    best_solution(:,prf) = Population(:,best_place);
    best_cost(prf) = sum(best_solution(:,prf))-Crowdness;
    
    test_ind = best_solution(:,prf);
    locations = Preference==test_ind;
    locations = sum (locations .* standard_matrix,2);
    class_distribution = histcounts(locations);
end

mean(best_cost)

