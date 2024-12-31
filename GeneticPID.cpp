struct Individual {
    float Kp, Ki, Kd;
    float fitness;

    Individual() : Kp(randomValue()), Ki(randomValue()), Kd(randomValue()), fitness(0) {}

    float randomValue() {
        return (rand() % 1000) / 100.0;  // Random value between 0 and 10
    }

    void evaluate(float setpoint, int steps) {
        fitness = costFunction(*this, setpoint, steps);
    }

    void crossover(Individual &other) {
        // Simple crossover: average Kp, Ki, Kd
        Kp = (Kp + other.Kp) / 2;
        Ki = (Ki + other.Ki) / 2;
        Kd = (Kd + other.Kd) / 2;
    }

    void mutate() {
        // Random mutation with a small chance
        if (rand() % 10 == 0) Kp += (rand() % 100) / 100.0 - 0.5;
        if (rand() % 10 == 0) Ki += (rand() % 100) / 100.0 - 0.5;
        if (rand() % 10 == 0) Kd += (rand() % 100) / 100.0 - 0.5;
    }
};

void geneticAlgorithm(float setpoint, int generations, int population_size, int steps) {
    // Initialize population
    std::vector<Individual> population(population_size);

    // Evolve over generations
    for (int gen = 0; gen < generations; ++gen) {
        // Evaluate fitness of each individual
        for (auto &ind : population) {
            ind.evaluate(setpoint, steps);
        }

        // Sort individuals based on fitness (lower is better)
        std::sort(population.begin(), population.end(), [](const Individual &a, const Individual &b) {
            return a.fitness < b.fitness;
        });

        // Select the top individuals for reproduction
        std::vector<Individual> new_population;
        for (int i = 0; i < population_size / 2; ++i) {
            Individual parent1 = population[i];
            Individual parent2 = population[i + 1];

            // Crossover
            parent1.crossover(parent2);

            // Mutate
            parent1.mutate();

            new_population.push_back(parent1);
            new_population.push_back(parent2);
        }

        // Replace old population with new population
        population = new_population;

        // Print best fitness of this generation
        std::cout << "Generation " << gen << " Best Fitness: " << population[0].fitness << std::endl;
    }

    // Return the best PID constants after the final generation
    PID best_pid = population[0];
    std::cout << "Best PID: Kp=" << best_pid.Kp << ", Ki=" << best_pid.Ki << ", Kd=" << best_pid.Kd << std::endl;
}


int main() {
    float setpoint = 100;  // Desired value (e.g., target position)
    int generations = 50;  // Number of generations
    int population_size = 20;  // Size of the population
    int steps = 100;  // Number of simulation steps per individual

    geneticAlgorithm(setpoint, generations, population_size, steps);

    return 0;
}
