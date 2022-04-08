import pygad
import numpy as np
import PID 

num = 1000

def callback_generation(ga_instance):
    if ga_instance.generations_completed % 10 == 0: 
        print("PID Generation = {generation}/{num}".format(generation=ga_instance.generations_completed,num=num))
        print("PID Fitness    = {fitness}".format(fitness=ga_instance.best_solution()[1]))
        # = ga_instance.best_solution()[0]
        #best_fit = st.main(best, plot = True)
        #print("Best Fitness    = {fitness}".format(fitness=best_fit))
    
def fitness_func(solution, solution_idx):
    #print(solution)
    # SOP between each w and X.
    fitness = 1/(PID.main(solution,speed)+1e-7)

    return fitness
   
def gain(inp_speed):
    global speed
    speed = inp_speed
    
    ga_instance = pygad.GA(num_generations= num,
                           num_parents_mating=150,
                           fitness_func=fitness_func,
                           sol_per_pop=180,
                           num_genes=3, # 4 in this example
                           suppress_warnings=True,
                           on_generation=callback_generation)
    try:
        ga_instance.run()
    
    except:
        pass
    
    finally:
        ga_instance.plot_result()
        
        solution, solution_fitness, _ = ga_instance.best_solution()
        print("Parameters of the best solution:\n{solution}".format(solution=solution), end="\n\n")
        print("Fitness value of the best solution:\n{solution_fitness}".format(solution_fitness=solution_fitness), end="\n\n")
    
        if ga_instance.best_solution_generation != -1:
            print("Best fitness value reached after {best_solution_generation} generations.".format(best_solution_generation=ga_instance.best_solution_generation))
    return solution
