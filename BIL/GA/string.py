import random
# Chromosomes are a string in population
POP_SIZE = 500 # Number of Chromosomes in our list
MUT_RATE = 0.1 # Rate at which our string will be changed
TARGET = 'siddharth' # Goal string
GENES = ' abcdefghijklmnopqrstuvwxyz' # Available options

def initialize_pop(TARGET):
    population = list()
    tar_len = len(TARGET) # Generating a chromosome with same length as target containing only options in genes

    for i in range(POP_SIZE):
        temp = list()
        for j in range(tar_len):
            temp.append(random.choice(GENES))
        population.append(temp)

    return population # initial population

def crossover(selected_chromo, CHROMO_LEN, population):
    # Randomly selected one parent from best chromosomes and one parent from the initial population. 
    # A crossover point defines the point from where info will be swapped among the parents to produce an offspring. 
    # This process adds diversity to our population.
    offspring_cross = []
    for i in range(int(POP_SIZE)):
        parent1 = random.choice(selected_chromo)
        parent2 = random.choice(population[:int(POP_SIZE*50)])

        p1 = parent1[0]
        p2 = parent2[0]

        crossover_point = random.randint(1, CHROMO_LEN-1)
        child =  p1[:crossover_point] + p2[crossover_point:]
        offspring_cross.extend([child])
    return offspring_cross

def mutate(offspring, MUT_RATE):
    # Mutation will randomly select a letter from each chromosome and replace it with another letter present in the GENES. 
    # The replacement probability depends on MUT_RATE and random number generated on each iteration
    mutated_offspring = []

    for arr in offspring:
        for i in range(len(arr)):
            if random.random() < MUT_RATE:
                arr[i] = random.choice(GENES)
        mutated_offspring.append(arr)
    return mutated_offspring

def selection(population, TARGET):
    # Returning only the top 50% of the population to avoid bad chromosomes from entering future population.
    sorted_chromo_pop = sorted(population, key= lambda x: x[1])
    return sorted_chromo_pop[:int(0.5*POP_SIZE)]

def fitness_cal(TARGET, chromo_from_pop): 
    # compares chaarcters in chromosome and target string for fitness evaluation. 
    # Higher value means far from target
    difference = 0
    for tar_char, chromo_char in zip(TARGET, chromo_from_pop):
        if tar_char != chromo_char:
            difference+=1
    
    return [chromo_from_pop, difference]

def replace(new_gen, population):
    # If new gen has a better (lesser) fitness score, it will replace the chromosome present in the initial population 
    # else it will continue to keep old chromosome.
    for _ in range(len(population)):
        if population[_][1] > new_gen[_][1]:
          population[_][0] = new_gen[_][0]
          population[_][1] = new_gen[_][1]
    return population

def main(POP_SIZE, MUT_RATE, TARGET, GENES):
    # 1) initialize population
    initial_population = initialize_pop(TARGET)
    found = False
    population = []
    generation = 1

    # 2) Calculating the fitness for the current population
    for _ in range(len(initial_population)):
        population.append(fitness_cal(TARGET, initial_population[_]))

    # now population has 2 things, [chromosome, fitness]
    # 3) now we loop until TARGET is found
    while not found:

      # 3.1) select best people from current population
      selected = selection(population, TARGET)

      # 3.2) mate parents to make new generation
      population = sorted(population, key= lambda x:x[1])
      crossovered = crossover(selected, len(TARGET), population)
            
      # 3.3) mutating the childeren to diversfy the new generation
      mutated = mutate(crossovered, MUT_RATE)

      new_gen = []
      for _ in mutated:
          new_gen.append(fitness_cal(TARGET, _))

      # 3.4) replacement of bad population with new generation
      # we sort here first to compare the least fit population with the most fit new_gen

      population = replace(new_gen, population)

      
      if (population[0][1] == 0):
        print('Target found')
        print('String: ' + str(population[0][0]) + ' Generation: ' + str(generation) + ' Fitness: ' + str(population[0][1]))
        break
      print('String: ' + str(population[0][0]) + ' Generation: ' + str(generation) + ' Fitness: ' + str(population[0][1]))
      generation+=1

main(POP_SIZE, MUT_RATE, TARGET, GENES)