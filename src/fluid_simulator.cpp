#include "fluid_simulator.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/random_number_generator.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <cmath>

using namespace godot;

FluidSimulator::FluidSimulator() {
    // Initialisation des paramètres par défaut
    gravity = Vector3(0, -9.81f, 0); // Gravité terrestre standard
    simulation_active = true;
    
    // Paramètres de référence
    max_particles = 1000;        // 1000 particules par défaut
    
    // Grille spatiale uniforme (résolution 20x20x20 pour 1m³)
    grid_resolution = 20;         // 20 cellules par dimension
    cell_size = BOX_SIZE / grid_resolution;  // Taille cellule = 1m / 20 = 5cm
    spatial_grid.resize(grid_resolution * grid_resolution * grid_resolution);
    
    // Buffers temporaires pour double buffering (seront redimensionnés dynamiquement)
    velocity_deltas.reserve(max_particles);
    position_deltas.reserve(max_particles);
    
    // Système de versement
    is_pouring = false;
    pour_timer = 0.0f;
    pour_duration = 0.0f;
    pour_particles_total = 0;
    pour_particles_added = 0;
    pour_rate = 0.0f;
    pour_pattern_index = 0;
    pour_liquid_name = "eau";
    pour_color = Color(0.3f, 0.6f, 1.0f, 1.0f); // Bleu pour l'eau
    pour_density = 1000.0f;
    pour_restitution = 0.4f;
    pour_particle_volume = 0.0f;
    pour_particle_radius = 0.0f;
    pour_particle_mass = 0.0f;
    
    // Paramètres de visibilité
    visibility_check_frequency = 10; // Vérifier toutes les 10 frames par défaut
    current_frame = 0;
    
    // Sous-étapes de simulation (physique pure)
    substeps = 10;                  // 10 substeps = stabilité suffisante
    
    // Viscosité et optimisation
    viscosity = 5.0f;               // Viscosité moyenne par défaut
    connection_radius = 2.5f;       // 2.5× le rayon moyen pour connexion
    interior_threshold = 18;        // 18+ connexions = particule intérieure (sur 26 max)
    optimize_interior = true;       // Activer l'optimisation par défaut
    
    // Caméra et occlusion culling 2D
    camera = nullptr;               // Pas de caméra par défaut
    occlusion_buffer_width = 256;   // Résolution par défaut 256x256 (meilleure précision)
    occlusion_buffer_height = 256;
    occlusion_buffer.resize(occlusion_buffer_width * occlusion_buffer_height);
    occlusion_valid = false;
    
    // Parallélisation de la physique
    num_threads = 0;                // 0 = auto-detect
    use_multithreading = true;      // Activé par défaut
    
    // Calcul automatique des propriétés de référence
    recalculate_reference_properties();
}

FluidSimulator::~FluidSimulator() {
    // Le std::vector gère automatiquement la mémoire
    particles.clear();
}

void FluidSimulator::recalculate_reference_properties() {
    // Calcul du volume de référence d'une particule : volume_total / nombre_particules
    // 1m³ / max_particles
    if (max_particles > 0) {
        reference_particle_volume = BOX_VOLUME / static_cast<float>(max_particles);
    } else {
        reference_particle_volume = 0.0f;
    }
    
    // Calcul du rayon de référence depuis le volume d'une sphère : V = (4/3) * π * r³
    // => r = (3V / 4π)^(1/3)
    const float PI = 3.14159265359f;
    if (reference_particle_volume > 0.0f) {
        reference_particle_radius = std::pow((3.0f * reference_particle_volume) / (4.0f * PI), 1.0f / 3.0f);
    } else {
        reference_particle_radius = 0.0f;
    }
}

void FluidSimulator::add_particle_in_pour_disc() {
    // Vérifie si on a atteint la limite
    if (max_particles > 0 && static_cast<int>(particles.size()) >= max_particles) {
        // Limite atteinte, arrêter le versement
        stop_pouring();
        UtilityFunctions::print("[FluidSimulator] Limite de particules atteinte (", max_particles, "). Versement arrêté.");
        return;
    }
    
    // Position au centre haut du cube (y = +0.5 pour le haut)
    float top_y = BOX_SIZE / 2.0f;
    
    // Distribution homogène en spirale de Fibonacci (méthode de Vogel)
    const float GOLDEN_ANGLE = 2.39996322972865332f;
    
    float angle = pour_pattern_index * GOLDEN_ANGLE;
    float normalized_radius = std::sqrt(static_cast<float>(pour_pattern_index % 1000) / 1000.0f);
    float radius = normalized_radius * POUR_DISC_RADIUS;
    
    float x = radius * std::cos(angle);
    float z = radius * std::sin(angle);
    
    Vector3 position(x, top_y, z);
    
    // Crée une particule avec les propriétés du liquide en cours de versement
    Particle p(position, pour_particle_mass, pour_particle_radius, pour_density, pour_restitution, pour_liquid_name, pour_color);
    particles.push_back(p);
    
    pour_pattern_index++;
}

void FluidSimulator::start_pouring_liquid(const String &liquid_name, const Color &color, float density, float restitution, float liters, float duration) {
    if (liters <= 0.0f || duration <= 0.0f || density <= 0.0f) {
        return; // Paramètres invalides
    }
    
    // Stocke les paramètres du liquide à verser
    pour_liquid_name = liquid_name;
    pour_color = color;
    pour_density = density;
    pour_restitution = restitution;
    
    // Calcul du volume en m³
    float volume_m3 = liters / 1000.0f;
    
    // Calcul des propriétés de ce liquide spécifique
    // Volume d'une particule pour ce liquide
    if (max_particles > 0) {
        pour_particle_volume = BOX_VOLUME / static_cast<float>(max_particles);
    } else {
        pour_particle_volume = reference_particle_volume;
    }
    
    // Rayon d'une particule pour ce liquide
    const float PI = 3.14159265359f;
    if (pour_particle_volume > 0.0f) {
        pour_particle_radius = std::pow((3.0f * pour_particle_volume) / (4.0f * PI), 1.0f / 3.0f);
    } else {
        pour_particle_radius = 0.01f; // Défaut
    }
    
    // Masse d'une particule pour ce liquide: masse = densité × volume
    pour_particle_mass = density * pour_particle_volume;
    
    // Nombre de particules à verser
    if (pour_particle_volume > 0.0f) {
        pour_particles_total = static_cast<int>(volume_m3 / pour_particle_volume);
    } else {
        pour_particles_total = 0;
        return;
    }
    
    // Initialisation du système de versement
    is_pouring = true;
    pour_timer = 0.0f;
    pour_duration = duration;
    pour_particles_added = 0;
    pour_rate = static_cast<float>(pour_particles_total) / duration;
    pour_pattern_index = 0;
}

void FluidSimulator::stop_pouring() {
    is_pouring = false;
    pour_timer = 0.0f;
    pour_particles_added = 0;
    pour_pattern_index = 0;
}

bool FluidSimulator::is_pouring_active() const {
    return is_pouring;
}

float FluidSimulator::get_pouring_progress() const {
    if (pour_particles_total == 0) {
        return 1.0f;
    }
    return static_cast<float>(pour_particles_added) / static_cast<float>(pour_particles_total);
}

void FluidSimulator::update_pouring(float delta_time) {
    if (!is_pouring) {
        return;
    }
    
    pour_timer += delta_time;
    
    // Calcule combien de particules doivent avoir été ajoutées à ce point
    int target_particles = static_cast<int>(pour_rate * pour_timer);
    
    // Limite au nombre total
    if (target_particles > pour_particles_total) {
        target_particles = pour_particles_total;
    }
    
    // Ajoute les particules manquantes
    while (pour_particles_added < target_particles) {
        add_particle_in_pour_disc();
        pour_particles_added++;
    }
    
    // Vérifie si le versement est terminé
    if (pour_particles_added >= pour_particles_total) {
        is_pouring = false;
    }
}

void FluidSimulator::clear_particles() {
    particles.clear();
}

int FluidSimulator::get_particle_count() const {
    return static_cast<int>(particles.size());
}

// ========== Parallélisation de la physique ==========

// Applique les forces et intègre la physique pour un range de particules (thread-safe)
void FluidSimulator::process_physics_range(size_t start, size_t end, float sub_dt) {
    for (size_t i = start; i < end; i++) {
        if (i >= particles.size()) break;
        
        Particle &p = particles[i];
        
        // Optimisation : particules intérieures ont moins de calculs
        if (optimize_interior && p.is_interior) {
            // Particule au centre : gravité réduite (compensée par la pression)
            PhysicsCalculations::reset_forces(p);
            PhysicsCalculations::apply_gravity(p, gravity * 0.5f); // 50% gravité
            PhysicsCalculations::integrate_euler(p, sub_dt);
            // Pas de contraintes de boîte (forcément à l'intérieur)
        } else {
            // Particule de surface ou optimisation désactivée : calcul complet
            PhysicsCalculations::reset_forces(p);
            PhysicsCalculations::apply_gravity(p, gravity);
            PhysicsCalculations::integrate_euler(p, sub_dt);
            PhysicsCalculations::apply_box_constraints(p, BOX_SIZE);
        }
    }
}

// Traite les collisions pour une partition de la grille spatiale (thread-safe)
// VERSION ANCIENNE (avec lock potentiel) - CONSERVÉE pour comparaison
void FluidSimulator::process_collisions_partition(int start_cell, int end_cell) {
    for (int cell_idx = start_cell; cell_idx < end_cell; cell_idx++) {
        if (cell_idx >= static_cast<int>(spatial_grid.size())) break;
        
        std::vector<size_t> &cell_particles = spatial_grid[cell_idx];
        
        if (cell_particles.empty()) continue;
        
        // Collisions within the same cell
        for (size_t i = 0; i < cell_particles.size(); i++) {
            for (size_t j = i + 1; j < cell_particles.size(); j++) {
                Particle &p1 = particles[cell_particles[i]];
                Particle &p2 = particles[cell_particles[j]];
                
                if (PhysicsCalculations::check_particle_collision(p1, p2)) {
                    PhysicsCalculations::resolve_particle_collision(p1, p2);
                }
            }
        }
        
        // Collisions with neighbor cells (only half to avoid duplicates)
        int z = cell_idx / (grid_resolution * grid_resolution);
        int y = (cell_idx / grid_resolution) % grid_resolution;
        int x = cell_idx % grid_resolution;
        
        const int neighbors[13][3] = {
            {1,0,0}, {-1,1,0}, {0,1,0}, {1,1,0},
            {-1,-1,1}, {0,-1,1}, {1,-1,1},
            {-1,0,1}, {0,0,1}, {1,0,1},
            {-1,1,1}, {0,1,1}, {1,1,1}
        };
        
        for (int n = 0; n < 13; n++) {
            int nx = x + neighbors[n][0];
            int ny = y + neighbors[n][1];
            int nz = z + neighbors[n][2];
            
            if (nx < 0 || nx >= grid_resolution || 
                ny < 0 || ny >= grid_resolution || 
                nz < 0 || nz >= grid_resolution) continue;
            
            int neighbor_idx = nx + ny * grid_resolution + nz * grid_resolution * grid_resolution;
            std::vector<size_t> &neighbor_particles = spatial_grid[neighbor_idx];
            
            for (size_t i : cell_particles) {
                for (size_t j : neighbor_particles) {
                    Particle &p1 = particles[i];
                    Particle &p2 = particles[j];
                    
                    if (PhysicsCalculations::check_particle_collision(p1, p2)) {
                        PhysicsCalculations::resolve_particle_collision(p1, p2);
                    }
                }
            }
        }
    }
}

// VERSION NOUVELLE (DOUBLE BUFFERING) - 100% LOCK-FREE !
// PHASE 1: Calcul des collisions (parallèle, chaque thread lit les données stables)
void FluidSimulator::process_collisions_calculate(int start_cell, int end_cell) {
    for (int cell_idx = start_cell; cell_idx < end_cell; cell_idx++) {
        if (cell_idx >= static_cast<int>(spatial_grid.size())) break;
        
        std::vector<size_t> &cell_particles = spatial_grid[cell_idx];
        if (cell_particles.empty()) continue;
        
        // Collisions au sein de la même cellule
        for (size_t i = 0; i < cell_particles.size(); i++) {
            for (size_t j = i + 1; j < cell_particles.size(); j++) {
                size_t idx1 = cell_particles[i];
                size_t idx2 = cell_particles[j];
                
                // LECTURE SEULE des particules (données stables)
                const Particle &p1 = particles[idx1];
                const Particle &p2 = particles[idx2];
                
                // Variables locales pour stocker les deltas
                Vector3 p1_vel_delta, p2_vel_delta, p1_pos_delta, p2_pos_delta;
                
                // Calcul de la collision (aucune écriture dans particles)
                PhysicsCalculations::calculate_collision_to_buffer(
                    p1, p2, 
                    p1_vel_delta, p2_vel_delta, 
                    p1_pos_delta, p2_pos_delta
                );
                
                // Accumulation des deltas dans les buffers temporaires
                // THREAD-SAFE: chaque particule n'est traitée que par un seul thread
                velocity_deltas[idx1] += p1_vel_delta;
                velocity_deltas[idx2] += p2_vel_delta;
                position_deltas[idx1] += p1_pos_delta;
                position_deltas[idx2] += p2_pos_delta;
            }
        }
        
        // Collisions avec les cellules voisines (half-space pour éviter duplicatas)
        int z = cell_idx / (grid_resolution * grid_resolution);
        int y = (cell_idx / grid_resolution) % grid_resolution;
        int x = cell_idx % grid_resolution;
        
        const int neighbors[13][3] = {
            {1,0,0}, {-1,1,0}, {0,1,0}, {1,1,0},
            {-1,-1,1}, {0,-1,1}, {1,-1,1},
            {-1,0,1}, {0,0,1}, {1,0,1},
            {-1,1,1}, {0,1,1}, {1,1,1}
        };
        
        for (int n = 0; n < 13; n++) {
            int nx = x + neighbors[n][0];
            int ny = y + neighbors[n][1];
            int nz = z + neighbors[n][2];
            
            if (nx < 0 || nx >= grid_resolution || 
                ny < 0 || ny >= grid_resolution || 
                nz < 0 || nz >= grid_resolution) continue;
            
            int neighbor_idx = nx + ny * grid_resolution + nz * grid_resolution * grid_resolution;
            std::vector<size_t> &neighbor_particles = spatial_grid[neighbor_idx];
            
            for (size_t idx1 : cell_particles) {
                for (size_t idx2 : neighbor_particles) {
                    // LECTURE SEULE
                    const Particle &p1 = particles[idx1];
                    const Particle &p2 = particles[idx2];
                    
                    Vector3 p1_vel_delta, p2_vel_delta, p1_pos_delta, p2_pos_delta;
                    
                    PhysicsCalculations::calculate_collision_to_buffer(
                        p1, p2, 
                        p1_vel_delta, p2_vel_delta, 
                        p1_pos_delta, p2_pos_delta
                    );
                    
                    // Accumulation thread-safe
                    velocity_deltas[idx1] += p1_vel_delta;
                    velocity_deltas[idx2] += p2_vel_delta;
                    position_deltas[idx1] += p1_pos_delta;
                    position_deltas[idx2] += p2_pos_delta;
                }
            }
        }
    }
}

// PHASE 2: Application des corrections (parallèle, chaque thread écrit sur ses particules)
void FluidSimulator::process_collisions_apply(size_t start, size_t end) {
    for (size_t i = start; i < end; i++) {
        if (i >= particles.size()) break;
        
        // Application des corrections accumulées
        particles[i].velocity += velocity_deltas[i];
        particles[i].position += position_deltas[i];
    }
}

// ========== Viscosité et optimisation ==========

void FluidSimulator::rebuild_viscosity_connections() {
    // Efface tous les voisins existants
    for (Particle &p : particles) {
        p.neighbors.clear();
        p.connection_count = 0;
    }
    
    // Utilise la grille spatiale pour trouver les voisins efficacement
    for (size_t i = 0; i < particles.size(); i++) {
        Particle &p = particles[i];
        int cell_idx = get_cell_index(p.position);
        
        // Cherche dans la cellule actuelle et les voisines
        int z = cell_idx / (grid_resolution * grid_resolution);
        int y = (cell_idx / grid_resolution) % grid_resolution;
        int x = cell_idx % grid_resolution;
        
        for (int dz = -1; dz <= 1; dz++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;
                    int nz = z + dz;
                    
                    if (nx < 0 || nx >= grid_resolution ||
                        ny < 0 || ny >= grid_resolution ||
                        nz < 0 || nz >= grid_resolution) continue;
                    
                    int neighbor_cell_idx = nx + ny * grid_resolution + nz * grid_resolution * grid_resolution;
                    
                    for (size_t j : spatial_grid[neighbor_cell_idx]) {
                        if (i >= j) continue; // Évite les doublons et auto-connexion
                        
                        Particle &neighbor = particles[j];
                        
                        // Vérifie si connexion possible (même liquide + distance)
                        if (PhysicsCalculations::check_viscosity_connection(p, neighbor, connection_radius)) {
                            p.neighbors.push_back(j);
                            neighbor.neighbors.push_back(i);
                            p.connection_count++;
                            neighbor.connection_count++;
                        }
                    }
                }
            }
        }
    }
}

void FluidSimulator::apply_viscosity_forces() {
    if (viscosity <= 0.0f) return;
    
    for (Particle &p : particles) {
        PhysicsCalculations::apply_viscosity_force(p, particles, viscosity);
    }
}

void FluidSimulator::update_interior_particles() {
    for (Particle &p : particles) {
        // Une particule est intérieure si elle a beaucoup de connexions
        p.is_interior = (p.connection_count >= interior_threshold);
    }
}

void FluidSimulator::update_simulation(float delta_time) {
    if (!simulation_active) {
        return;
    }
    
    // Mise à jour du système de versement progressif
    update_pouring(delta_time);
    
    // Diviser le temps en sous-étapes
    float sub_dt = delta_time / static_cast<float>(substeps);
    
    // Déterminer si on utilise le multi-threading
    bool should_use_mt = use_multithreading && particles.size() > 1000; // Seuil : 1000 particules
    int active_threads = should_use_mt ? get_num_threads() : 1;
    
    // Effectuer N sous-étapes de simulation (physique pure)
    for (int substep = 0; substep < substeps; substep++) {
        // PHASE 1 : Mise à jour de la physique individuelle
        if (should_use_mt && active_threads > 1) {
            // ========== MODE PARALLÈLE ==========
            std::vector<std::thread> threads;
            size_t particles_per_thread = particles.size() / active_threads;
            
            for (int t = 0; t < active_threads; t++) {
                size_t start = t * particles_per_thread;
                size_t end = (t == active_threads - 1) ? particles.size() : (t + 1) * particles_per_thread;
                
                threads.emplace_back([this, start, end, sub_dt]() {
                    this->process_physics_range(start, end, sub_dt);
                });
            }
            
            // Attendre que tous les threads se terminent
            for (auto& thread : threads) {
                thread.join();
            }
        } else {
            // ========== MODE SÉQUENTIEL ==========
            for (size_t i = 0; i < particles.size(); i++) {
                Particle &p = particles[i];
                
                PhysicsCalculations::reset_forces(p);
                PhysicsCalculations::apply_gravity(p, gravity);
                PhysicsCalculations::integrate_euler(p, sub_dt);
                PhysicsCalculations::apply_box_constraints(p, BOX_SIZE);
            }
        }
        
        // PHASE 2 : Gérer les collisions entre particules (avec DOUBLE BUFFERING pour lock-free)
        if (should_use_mt && active_threads > 1) {
            // ========== COLLISIONS PARALLÈLES (DOUBLE BUFFERING) ==========
            rebuild_spatial_grid(); // Reconstruire la grille (séquentiel)
            
            // Reconstruire les connexions de viscosité (utilise la grille spatiale)
            if (viscosity > 0.0f) {
                rebuild_viscosity_connections();
                update_interior_particles();
            }
            
            // Préparer les buffers de deltas (réinitialiser à zéro)
            velocity_deltas.assign(particles.size(), Vector3(0, 0, 0));
            position_deltas.assign(particles.size(), Vector3(0, 0, 0));
            
            // PHASE 2.1 : Calcul des collisions (100% parallèle, LECTURE SEULE)
            {
                std::vector<std::thread> threads;
                int total_cells = static_cast<int>(spatial_grid.size());
                int cells_per_thread = total_cells / active_threads;
                
                for (int t = 0; t < active_threads; t++) {
                    int start_cell = t * cells_per_thread;
                    int end_cell = (t == active_threads - 1) ? total_cells : (t + 1) * cells_per_thread;
                    
                    threads.emplace_back([this, start_cell, end_cell]() {
                        this->process_collisions_calculate(start_cell, end_cell);
                    });
                }
                
                // Attendre la fin du calcul
                for (auto& thread : threads) {
                    thread.join();
                }
            }
            
            // PHASE 2.2 : Application des deltas (100% parallèle, ÉCRITURE)
            {
                std::vector<std::thread> threads;
                size_t particles_per_thread = particles.size() / active_threads;
                
                for (int t = 0; t < active_threads; t++) {
                    size_t start = t * particles_per_thread;
                    size_t end = (t == active_threads - 1) ? particles.size() : (t + 1) * particles_per_thread;
                    
                    threads.emplace_back([this, start, end]() {
                        this->process_collisions_apply(start, end);
                    });
                }
                
                // Attendre la fin de l'application
                for (auto& thread : threads) {
                    thread.join();
                }
            }
        } else {
            // ========== COLLISIONS SÉQUENTIELLES ==========
            rebuild_spatial_grid();
            
            // Reconstruire les connexions de viscosité
            if (viscosity > 0.0f) {
                rebuild_viscosity_connections();
                update_interior_particles();
            }
            
            handle_particle_collisions();
        }
        
        // PHASE 3 : Appliquer les forces de viscosité (après les collisions)
        if (viscosity > 0.0f) {
            apply_viscosity_forces();
        }
    }
    
    // 9. Mettre à jour la visibilité (détection de surface) - pas chaque frame
    current_frame++;
    if (current_frame >= visibility_check_frequency) {
        update_particle_visibility();
        current_frame = 0;
    }
}

void FluidSimulator::set_simulation_active(bool active) {
    simulation_active = active;
}

bool FluidSimulator::is_simulation_active() const {
    return simulation_active;
}

float FluidSimulator::get_box_size() const {
    return BOX_SIZE; // Toujours 1m³
}

float FluidSimulator::get_box_volume_liters() const {
    return BOX_VOLUME * 1000.0f; // 1m³ = 1000 litres
}

void FluidSimulator::set_gravity(Vector3 g) {
    gravity = g;
}

Vector3 FluidSimulator::get_gravity() const {
    return gravity;
}

void FluidSimulator::set_max_particles(int count) {
    if (count > 0) {
        max_particles = count;
        recalculate_reference_properties(); // Recalcule seulement les valeurs de référence
    }
}

int FluidSimulator::get_max_particles() const {
    return max_particles;
}

float FluidSimulator::get_reference_particle_volume() const {
    return reference_particle_volume;
}

float FluidSimulator::get_reference_particle_radius() const {
    return reference_particle_radius;
}

// Reconstruit la grille spatiale (clear + remplissage, pas de réallocation)
void FluidSimulator::rebuild_spatial_grid() {
    // Clear toutes les cellules (garde la mémoire allouée)
    for (auto &cell : spatial_grid) {
        cell.clear();
    }
    
    // Insère chaque particule dans sa cellule
    for (size_t i = 0; i < particles.size(); i++) {
        int cell_index = get_cell_index(particles[i].position);
        spatial_grid[cell_index].push_back(i);
    }
}

void FluidSimulator::handle_particle_collisions() {
    // Reconstruit la grille spatiale (clear + remplissage, pas de réallocation)
    rebuild_spatial_grid();
    
    // Pour chaque cellule de la grille
    for (int cell_idx = 0; cell_idx < static_cast<int>(spatial_grid.size()); cell_idx++) {
        std::vector<size_t> &cell_particles = spatial_grid[cell_idx];
        
        if (cell_particles.empty()) continue;
        
        // Collision entre particules de la même cellule
        for (size_t i = 0; i < cell_particles.size(); i++) {
            for (size_t j = i + 1; j < cell_particles.size(); j++) {
                Particle &p1 = particles[cell_particles[i]];
                Particle &p2 = particles[cell_particles[j]];
                
                if (PhysicsCalculations::check_particle_collision(p1, p2)) {
                    PhysicsCalculations::resolve_particle_collision(p1, p2);
                }
            }
        }
        
        // Collision avec cellules voisines (seulement la moitié pour éviter doublons)
        // Calcul des coordonnées 3D de la cellule actuelle
        int z = cell_idx / (grid_resolution * grid_resolution);
        int y = (cell_idx / grid_resolution) % grid_resolution;
        int x = cell_idx % grid_resolution;
        
        // Vérifier les 13 cellules voisines (moitié de 26) pour éviter doublons
        const int neighbors[13][3] = {
            {1,0,0}, {-1,1,0}, {0,1,0}, {1,1,0},
            {-1,-1,1}, {0,-1,1}, {1,-1,1},
            {-1,0,1}, {0,0,1}, {1,0,1},
            {-1,1,1}, {0,1,1}, {1,1,1}
        };
        
        for (int n = 0; n < 13; n++) {
            int nx = x + neighbors[n][0];
            int ny = y + neighbors[n][1];
            int nz = z + neighbors[n][2];
            
            // Vérifier les limites
            if (nx < 0 || nx >= grid_resolution || 
                ny < 0 || ny >= grid_resolution || 
                nz < 0 || nz >= grid_resolution) continue;
            
            int neighbor_idx = nx + ny * grid_resolution + nz * grid_resolution * grid_resolution;
            std::vector<size_t> &neighbor_particles = spatial_grid[neighbor_idx];
            
            // Collision entre cellule courante et cellule voisine
            for (size_t i : cell_particles) {
                for (size_t j : neighbor_particles) {
                    Particle &p1 = particles[i];
                    Particle &p2 = particles[j];
                    
                    if (PhysicsCalculations::check_particle_collision(p1, p2)) {
                        PhysicsCalculations::resolve_particle_collision(p1, p2);
                    }
                }
            }
        }
    }
}

Vector3 FluidSimulator::get_particle_position(int index) const {
    if (index >= 0 && index < static_cast<int>(particles.size())) {
        return particles[index].position;
    }
    return Vector3(0, 0, 0);
}

Vector3 FluidSimulator::get_particle_velocity(int index) const {
    if (index >= 0 && index < static_cast<int>(particles.size())) {
        return particles[index].velocity;
    }
    return Vector3(0, 0, 0);
}

String FluidSimulator::get_particle_liquid_name(int index) const {
    if (index >= 0 && index < static_cast<int>(particles.size())) {
        return particles[index].liquid_name;
    }
    return "unknown";
}

float FluidSimulator::get_particle_density(int index) const {
    if (index >= 0 && index < static_cast<int>(particles.size())) {
        return particles[index].density;
    }
    return 0.0f;
}

Color FluidSimulator::get_particle_color(int index) const {
    if (index >= 0 && index < static_cast<int>(particles.size())) {
        return particles[index].color;
    }
    return Color(1.0f, 1.0f, 1.0f, 1.0f); // Blanc par défaut
}

bool FluidSimulator::is_particle_visible(int index) const {
    if (index >= 0 && index < static_cast<int>(particles.size())) {
        return particles[index].visible;
    }
    return false;
}

int FluidSimulator::get_visible_particle_count() const {
    int count = 0;
    for (const Particle &p : particles) {
        if (p.visible) {
            count++;
        }
    }
    return count;
}

void FluidSimulator::set_visibility_check_frequency(int frequency) {
    if (frequency > 0) {
        visibility_check_frequency = frequency;
    }
}

int FluidSimulator::get_visibility_check_frequency() const {
    return visibility_check_frequency;
}

void FluidSimulator::set_substeps(int steps) {
    substeps = Math::max(steps, 1);
}

int FluidSimulator::get_substeps() const {
    return substeps;
}

void FluidSimulator::set_viscosity(float visc) {
    viscosity = Math::max(visc, 0.0f);
}

float FluidSimulator::get_viscosity() const {
    return viscosity;
}

void FluidSimulator::set_connection_radius(float radius) {
    connection_radius = Math::max(radius, 1.0f);
}

float FluidSimulator::get_connection_radius() const {
    return connection_radius;
}

void FluidSimulator::set_interior_threshold(int threshold) {
    interior_threshold = Math::max(threshold, 0);
}

int FluidSimulator::get_interior_threshold() const {
    return interior_threshold;
}

void FluidSimulator::set_optimize_interior(bool enable) {
    optimize_interior = enable;
}

bool FluidSimulator::get_optimize_interior() const {
    return optimize_interior;
}

void FluidSimulator::set_camera(Node3D* cam) {
    camera = cam;
    occlusion_valid = false; // Invalide le buffer quand on change de caméra
}

Node3D* FluidSimulator::get_camera() const {
    return camera;
}

void FluidSimulator::set_occlusion_buffer_resolution(int width, int height) {
    if (width > 0 && height > 0) {
        occlusion_buffer_width = width;
        occlusion_buffer_height = height;
        occlusion_buffer.resize(width * height);
        occlusion_valid = false;
    }
}

int FluidSimulator::get_occlusion_buffer_width() const {
    return occlusion_buffer_width;
}

int FluidSimulator::get_occlusion_buffer_height() const {
    return occlusion_buffer_height;
}

void FluidSimulator::set_num_threads(int threads) {
    if (threads == -1) {
        // -1 = désactiver le multi-threading
        num_threads = 1;
        use_multithreading = false;
    } else if (threads == 0) {
        // 0 = auto-detect
        num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) num_threads = 4; // Fallback si détection échoue
    } else {
        num_threads = (threads > 0) ? threads : 1;
    }
}

int FluidSimulator::get_num_threads() const {
    if (num_threads == 0) {
        // Auto-detect à la volée
        int detected = std::thread::hardware_concurrency();
        return (detected > 0) ? detected : 4;
    }
    return num_threads;
}

void FluidSimulator::set_use_multithreading(bool enable) {
    use_multithreading = enable;
}

bool FluidSimulator::get_use_multithreading() const {
    return use_multithreading;
}

// Projette une position 3D vers les coordonnées écran 2D
bool FluidSimulator::project_to_screen(const Vector3& world_pos, const Transform3D& view_matrix, 
                                        const Transform3D& projection_matrix,
                                        int& screen_x, int& screen_y, float& depth) const {
    // Transformation en espace vue
    Vector3 view_pos = view_matrix.xform(world_pos);
    depth = -view_pos.z; // Profondeur (Z négatif en espace vue)
    
    // Clip si derrière la caméra
    if (depth <= 0.0f) {
        return false;
    }
    
    // Projection en espace clip
    Vector3 clip_pos = projection_matrix.xform(view_pos);
    
    // Division de perspective
    if (std::abs(clip_pos.z) < 0.0001f) {
        return false;
    }
    
    float ndc_x = clip_pos.x / clip_pos.z;
    float ndc_y = clip_pos.y / clip_pos.z;
    
    // Clip si hors du frustum
    if (ndc_x < -1.0f || ndc_x > 1.0f || ndc_y < -1.0f || ndc_y > 1.0f) {
        return false;
    }
    
    // Conversion NDC [-1, 1] vers coordonnées écran [0, width/height]
    screen_x = static_cast<int>((ndc_x + 1.0f) * 0.5f * occlusion_buffer_width);
    screen_y = static_cast<int>((1.0f - ndc_y) * 0.5f * occlusion_buffer_height); // Y inversé
    
    // Clamp pour sécurité
    screen_x = (screen_x < 0) ? 0 : (screen_x >= occlusion_buffer_width) ? occlusion_buffer_width - 1 : screen_x;
    screen_y = (screen_y < 0) ? 0 : (screen_y >= occlusion_buffer_height) ? occlusion_buffer_height - 1 : screen_y;
    
    return true;
}

// Rasterise un disque (particule) sur le buffer d'occlusion
void FluidSimulator::rasterize_particle_disk(size_t particle_index, int center_x, int center_y, 
                                               int radius_pixels, float depth) {
    rasterize_particle_disk_to_buffer(occlusion_buffer, particle_index, center_x, center_y, radius_pixels, depth);
}

// Rasterise un disque dans un buffer spécifique (thread-safe)
void FluidSimulator::rasterize_particle_disk_to_buffer(std::vector<ScreenPixel>& buffer, 
                                                         size_t particle_index, int center_x, int center_y, 
                                                         int radius_pixels, float depth) const {
    // Parcourir le carré englobant le disque
    int min_x = (center_x - radius_pixels < 0) ? 0 : center_x - radius_pixels;
    int max_x = (center_x + radius_pixels >= occlusion_buffer_width) ? occlusion_buffer_width - 1 : center_x + radius_pixels;
    int min_y = (center_y - radius_pixels < 0) ? 0 : center_y - radius_pixels;
    int max_y = (center_y + radius_pixels >= occlusion_buffer_height) ? occlusion_buffer_height - 1 : center_y + radius_pixels;
    
    int radius_sq = radius_pixels * radius_pixels;
    
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            // Test si le pixel est dans le disque
            int dx = x - center_x;
            int dy = y - center_y;
            int dist_sq = dx * dx + dy * dy;
            
            if (dist_sq <= radius_sq) {
                // Pixel dans le disque
                int buffer_index = y * occlusion_buffer_width + x;
                
                // Garder seulement la particule la plus proche
                if (depth < buffer[buffer_index].depth) {
                    buffer[buffer_index].particle_index = particle_index;
                    buffer[buffer_index].depth = depth;
                }
            }
        }
    }
}

// Rasterise un range de particules dans un buffer temporaire (thread-safe)
void FluidSimulator::rasterize_particles_range(size_t start, size_t end, 
                                                const Transform3D& view_matrix,
                                                const Transform3D& projection_matrix,
                                                float particle_radius_world, float f,
                                                std::vector<ScreenPixel>& temp_buffer) const {
    for (size_t i = start; i < end && i < particles.size(); i++) {
        int screen_x, screen_y;
        float depth;
        
        if (project_to_screen(particles[i].position, view_matrix, projection_matrix, 
                              screen_x, screen_y, depth)) {
            // Calculer le rayon du disque à l'écran
            int radius_pixels = static_cast<int>((particle_radius_world / depth) * occlusion_buffer_width * 0.5f * f);
            radius_pixels = (radius_pixels < 1) ? 1 : radius_pixels;
            
            // Rasteriser le disque dans le buffer temporaire
            rasterize_particle_disk_to_buffer(temp_buffer, i, screen_x, screen_y, radius_pixels, depth);
        }
    }
}

// Fusionne un buffer temporaire dans le buffer principal (avec Z-test)
void FluidSimulator::merge_buffer(const std::vector<ScreenPixel>& temp_buffer) const {
    FluidSimulator* mutable_this = const_cast<FluidSimulator*>(this);
    
    for (size_t i = 0; i < temp_buffer.size(); i++) {
        const ScreenPixel& temp_pixel = temp_buffer[i];
        
        // Si ce pixel a une particule dans le buffer temporaire
        if (temp_pixel.particle_index != SIZE_MAX) {
            // Comparer avec le buffer principal
            if (temp_pixel.depth < mutable_this->occlusion_buffer[i].depth) {
                mutable_this->occlusion_buffer[i].particle_index = temp_pixel.particle_index;
                mutable_this->occlusion_buffer[i].depth = temp_pixel.depth;
            }
        }
    }
}

// Reconstruit le buffer d'occlusion à partir de toutes les particules
void FluidSimulator::rebuild_occlusion_buffer() const {
    if (!camera || particles.empty()) {
        occlusion_valid = false;
        return;
    }
    
    FluidSimulator* mutable_this = const_cast<FluidSimulator*>(this);
    
    // Réinitialiser le buffer
    for (auto& pixel : occlusion_buffer) {
        pixel.particle_index = SIZE_MAX;
        pixel.depth = FLT_MAX;
    }
    
    // Récupérer les matrices de transformation
    Transform3D view_matrix = camera->get_global_transform().affine_inverse();
    
    // Construire une matrice de projection simple (via call car Camera3D n'est pas bindé)
    float fov = (float)camera->call("get_fov");
    float aspect = (float)occlusion_buffer_width / (float)occlusion_buffer_height;
    float near = (float)camera->call("get_near");
    float far = (float)camera->call("get_far");
    
    float fov_rad = fov * (3.14159265359f / 180.0f);
    float tan_half_fov = std::tan(fov_rad * 0.5f);
    
    // Matrice de projection perspective simple
    Transform3D projection_matrix;
    float f = 1.0f / tan_half_fov;
    projection_matrix.basis.rows[0][0] = f / aspect;
    projection_matrix.basis.rows[1][1] = f;
    projection_matrix.basis.rows[2][2] = (far + near) / (near - far);
    projection_matrix.origin.z = (2.0f * far * near) / (near - far);
    
    // Estimer la taille d'une particule à l'écran (en pixels)
    float particle_radius_world = (particles.size() > 0) ? particles[0].radius : reference_particle_radius;
    
    // Déterminer si on utilise le multi-threading
    bool should_use_mt = use_multithreading && particles.size() > 1000;
    int active_threads = should_use_mt ? get_num_threads() : 1;
    
    if (should_use_mt && active_threads > 1) {
        // ========== MODE PARALLÈLE ==========
        
        // Créer un buffer temporaire par thread
        int buffer_size = occlusion_buffer_width * occlusion_buffer_height;
        std::vector<std::vector<ScreenPixel>> thread_buffers(active_threads);
        
        for (int t = 0; t < active_threads; t++) {
            thread_buffers[t].resize(buffer_size);
            // Initialiser le buffer temporaire
            for (auto& pixel : thread_buffers[t]) {
                pixel.particle_index = SIZE_MAX;
                pixel.depth = FLT_MAX;
            }
        }
        
        // Rasteriser les particules en parallèle
        std::vector<std::thread> threads;
        size_t particles_per_thread = particles.size() / active_threads;
        
        for (int t = 0; t < active_threads; t++) {
            size_t start = t * particles_per_thread;
            size_t end = (t == active_threads - 1) ? particles.size() : (t + 1) * particles_per_thread;
            
            threads.emplace_back([this, start, end, &view_matrix, &projection_matrix, 
                                 particle_radius_world, f, &thread_buffers, t]() {
                this->rasterize_particles_range(start, end, view_matrix, projection_matrix, 
                                               particle_radius_world, f, thread_buffers[t]);
            });
        }
        
        // Attendre que tous les threads terminent la rasterisation
        for (auto& thread : threads) {
            thread.join();
        }
        
        // Fusionner les buffers temporaires dans le buffer principal
        for (const auto& temp_buffer : thread_buffers) {
            mutable_this->merge_buffer(temp_buffer);
        }
        
    } else {
        // ========== MODE SÉQUENTIEL ==========
        for (size_t i = 0; i < particles.size(); i++) {
            int screen_x, screen_y;
            float depth;
            
            if (project_to_screen(particles[i].position, view_matrix, projection_matrix, 
                                  screen_x, screen_y, depth)) {
                int radius_pixels = static_cast<int>((particle_radius_world / depth) * occlusion_buffer_width * 0.5f * f);
                radius_pixels = (radius_pixels < 1) ? 1 : radius_pixels;
                
                mutable_this->rasterize_particle_disk(i, screen_x, screen_y, radius_pixels, depth);
            }
        }
    }
    
    occlusion_valid = true;
}

// Vérifie si une particule est visible (présente dans le buffer d'occlusion)
bool FluidSimulator::is_particle_on_surface(size_t particle_index) const {
    if (particle_index >= particles.size()) {
        return false;
    }
    
    // Si pas de caméra, afficher toutes les particules
    if (!camera) {
        return true;
    }
    
    // Reconstruire le buffer si invalide
    if (!occlusion_valid) {
        rebuild_occlusion_buffer();
    }
    
    // Si le buffer est toujours invalide (pas de caméra ou erreur), afficher tout
    if (!occlusion_valid) {
        return true;
    }
    
    // Parcourir le buffer d'occlusion pour voir si cette particule est visible
    for (const auto& pixel : occlusion_buffer) {
        if (pixel.particle_index == particle_index) {
            return true; // Particule trouvée dans le buffer = visible
        }
    }
    
    return false; // Particule non trouvée = occultée
}

void FluidSimulator::update_particle_visibility() {
    // Invalide le buffer d'occlusion (la caméra a pu bouger)
    occlusion_valid = false;
    
    // Reconstruit le buffer d'occlusion
    rebuild_occlusion_buffer();
    
    // Met à jour la visibilité de chaque particule
    for (size_t i = 0; i < particles.size(); i++) {
        particles[i].visible = is_particle_on_surface(i);
    }
}

float FluidSimulator::get_total_kinetic_energy() const {
    float total_energy = 0.0f;
    for (const Particle &p : particles) {
        total_energy += PhysicsCalculations::calculate_kinetic_energy(p);
    }
    return total_energy;
}

void FluidSimulator::_ready() {
    // Appelé quand le node entre dans la scène
    // Vous pouvez ajouter des particules de test ici si vous voulez
}

void FluidSimulator::_physics_process(double delta) {
    // Appelé chaque frame physique (60 FPS par défaut)
    // On convertit le double en float pour notre simulation
    update_simulation(static_cast<float>(delta));
}

void FluidSimulator::_bind_methods() {
    // Liaison des méthodes pour les rendre disponibles dans Godot
    
    // Gestion des particules
    ClassDB::bind_method(D_METHOD("start_pouring_liquid", "liquid_name", "color", "density", "restitution", "liters", "duration"), &FluidSimulator::start_pouring_liquid);
    ClassDB::bind_method(D_METHOD("stop_pouring"), &FluidSimulator::stop_pouring);
    ClassDB::bind_method(D_METHOD("is_pouring_active"), &FluidSimulator::is_pouring_active);
    ClassDB::bind_method(D_METHOD("get_pouring_progress"), &FluidSimulator::get_pouring_progress);
    ClassDB::bind_method(D_METHOD("clear_particles"), &FluidSimulator::clear_particles);
    ClassDB::bind_method(D_METHOD("get_particle_count"), &FluidSimulator::get_particle_count);
    
    // Simulation
    ClassDB::bind_method(D_METHOD("update_simulation", "delta_time"), &FluidSimulator::update_simulation);
    ClassDB::bind_method(D_METHOD("set_simulation_active", "active"), &FluidSimulator::set_simulation_active);
    ClassDB::bind_method(D_METHOD("is_simulation_active"), &FluidSimulator::is_simulation_active);
    
    // Taille de la boîte (lecture seule)
    ClassDB::bind_method(D_METHOD("get_box_size"), &FluidSimulator::get_box_size);
    ClassDB::bind_method(D_METHOD("get_box_volume_liters"), &FluidSimulator::get_box_volume_liters);
    
    // Propriétés - Gravité
    ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &FluidSimulator::set_gravity);
    ClassDB::bind_method(D_METHOD("get_gravity"), &FluidSimulator::get_gravity);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity"), "set_gravity", "get_gravity");
    
    // Propriétés - Nombre maximum de particules (pour référence)
    ClassDB::bind_method(D_METHOD("set_max_particles", "count"), &FluidSimulator::set_max_particles);
    ClassDB::bind_method(D_METHOD("get_max_particles"), &FluidSimulator::get_max_particles);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_particles"), "set_max_particles", "get_max_particles");
    
    // Propriétés de référence calculées (lecture seule)
    ClassDB::bind_method(D_METHOD("get_reference_particle_volume"), &FluidSimulator::get_reference_particle_volume);
    ClassDB::bind_method(D_METHOD("get_reference_particle_radius"), &FluidSimulator::get_reference_particle_radius);
    
    // Informations sur les particules
    ClassDB::bind_method(D_METHOD("get_particle_position", "index"), &FluidSimulator::get_particle_position);
    ClassDB::bind_method(D_METHOD("get_particle_velocity", "index"), &FluidSimulator::get_particle_velocity);
    ClassDB::bind_method(D_METHOD("get_particle_liquid_name", "index"), &FluidSimulator::get_particle_liquid_name);
    ClassDB::bind_method(D_METHOD("get_particle_density", "index"), &FluidSimulator::get_particle_density);
    ClassDB::bind_method(D_METHOD("get_particle_color", "index"), &FluidSimulator::get_particle_color);
    ClassDB::bind_method(D_METHOD("is_particle_visible", "index"), &FluidSimulator::is_particle_visible);
    ClassDB::bind_method(D_METHOD("get_visible_particle_count"), &FluidSimulator::get_visible_particle_count);
    ClassDB::bind_method(D_METHOD("get_total_kinetic_energy"), &FluidSimulator::get_total_kinetic_energy);
    
    // Paramètres de visibilité
    ClassDB::bind_method(D_METHOD("set_visibility_check_frequency", "frequency"), &FluidSimulator::set_visibility_check_frequency);
    ClassDB::bind_method(D_METHOD("get_visibility_check_frequency"), &FluidSimulator::get_visibility_check_frequency);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "visibility_check_frequency"), "set_visibility_check_frequency", "get_visibility_check_frequency");
    
    // Paramètres de substeps (physique pure)
    ClassDB::bind_method(D_METHOD("set_substeps", "steps"), &FluidSimulator::set_substeps);
    ClassDB::bind_method(D_METHOD("get_substeps"), &FluidSimulator::get_substeps);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "substeps", PROPERTY_HINT_RANGE, "1,50,1"), "set_substeps", "get_substeps");
    
    // Paramètres de viscosité
    ClassDB::bind_method(D_METHOD("set_viscosity", "viscosity"), &FluidSimulator::set_viscosity);
    ClassDB::bind_method(D_METHOD("get_viscosity"), &FluidSimulator::get_viscosity);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "viscosity", PROPERTY_HINT_RANGE, "0.0,100.0,0.1"), "set_viscosity", "get_viscosity");
    
    ClassDB::bind_method(D_METHOD("set_connection_radius", "radius"), &FluidSimulator::set_connection_radius);
    ClassDB::bind_method(D_METHOD("get_connection_radius"), &FluidSimulator::get_connection_radius);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "connection_radius", PROPERTY_HINT_RANGE, "1.0,5.0,0.1"), "set_connection_radius", "get_connection_radius");
    
    ClassDB::bind_method(D_METHOD("set_interior_threshold", "threshold"), &FluidSimulator::set_interior_threshold);
    ClassDB::bind_method(D_METHOD("get_interior_threshold"), &FluidSimulator::get_interior_threshold);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "interior_threshold", PROPERTY_HINT_RANGE, "0,26,1"), "set_interior_threshold", "get_interior_threshold");
    
    ClassDB::bind_method(D_METHOD("set_optimize_interior", "enable"), &FluidSimulator::set_optimize_interior);
    ClassDB::bind_method(D_METHOD("get_optimize_interior"), &FluidSimulator::get_optimize_interior);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "optimize_interior"), "set_optimize_interior", "get_optimize_interior");
    
    // Caméra pour occlusion culling 2D (software rasterizer)
    ClassDB::bind_method(D_METHOD("set_camera", "camera"), &FluidSimulator::set_camera);
    ClassDB::bind_method(D_METHOD("get_camera"), &FluidSimulator::get_camera);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "camera", PROPERTY_HINT_NODE_TYPE, "Camera3D"), "set_camera", "get_camera");
    
    // Résolution du buffer d'occlusion (défaut: 128x128, plus élevé = plus précis mais plus lent)
    ClassDB::bind_method(D_METHOD("set_occlusion_buffer_resolution", "width", "height"), &FluidSimulator::set_occlusion_buffer_resolution);
    ClassDB::bind_method(D_METHOD("get_occlusion_buffer_width"), &FluidSimulator::get_occlusion_buffer_width);
    ClassDB::bind_method(D_METHOD("get_occlusion_buffer_height"), &FluidSimulator::get_occlusion_buffer_height);
    
    // Parallélisation de la physique
    ClassDB::bind_method(D_METHOD("set_num_threads", "threads"), &FluidSimulator::set_num_threads);
    ClassDB::bind_method(D_METHOD("get_num_threads"), &FluidSimulator::get_num_threads);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "num_threads", PROPERTY_HINT_RANGE, "-1,16,1"), "set_num_threads", "get_num_threads");
    
    ClassDB::bind_method(D_METHOD("set_use_multithreading", "enable"), &FluidSimulator::set_use_multithreading);
    ClassDB::bind_method(D_METHOD("get_use_multithreading"), &FluidSimulator::get_use_multithreading);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_multithreading"), "set_use_multithreading", "get_use_multithreading");
}


