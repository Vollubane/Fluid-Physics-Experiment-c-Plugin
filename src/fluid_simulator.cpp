#include "fluid_simulator.hpp"
#include "particle_physics.hpp"
#include "marching_cubes.hpp"
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
    
    // Hachage spatial (cellule 2x plus grande que le diamètre moyen des particules)
    spatial_hash_cell_size = 0.02f; // 2cm par défaut, sera ajusté automatiquement
    
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
    
    // Paramètres de stabilité
    velocity_damping = 0.85f;      // 15% de friction par défaut (85% de conservation) - fluide visqueux
    sleep_threshold = 0.005f;      // Mettre au repos si vitesse < 5mm/s (plus permissif)
    
    // Paramètres de reconstruction de surface
    surface_mesh_enabled = false;  // Désactivé par défaut (utiliser MultiMesh pour les particules)
    surface_threshold = 0.6f;      // Seuil pour la surface (ajustable selon la densité des particules)
    surface_grid_resolution = 0.02f; // Grille de 2cm par cellule
    
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
    
    // Ajuste la taille des cellules du hachage spatial
    // Cellule = 4 × rayon de référence pour efficacité optimale
    spatial_hash_cell_size = reference_particle_radius * 4.0f;
    if (spatial_hash_cell_size < 0.01f) {
        spatial_hash_cell_size = 0.01f; // Minimum 1cm
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

void FluidSimulator::update_simulation(float delta_time) {
    if (!simulation_active) {
        return;
    }
    
    // Mise à jour du système de versement progressif
    update_pouring(delta_time);
    
    // Pour chaque particule
    for (size_t i = 0; i < particles.size(); i++) {
        Particle &p = particles[i];
        
        // 1. Réinitialiser les forces
        PhysicsCalculations::reset_forces(p);
        
        // 2. Appliquer la gravité (toujours active, c'est une vraie force)
        PhysicsCalculations::apply_gravity(p, gravity);
        
        // 3. Intégrer la physique (mise à jour position et vélocité)
        PhysicsCalculations::integrate_euler(p, delta_time);
        
        // 4. Appliquer l'amortissement de la vitesse (friction de l'air + viscosité du fluide)
        p.velocity *= velocity_damping;
        
        // 5. Mettre au repos si la vitesse est trop faible (évite les micro-mouvements)
        if (p.velocity.length() < sleep_threshold) {
            p.velocity = Vector3(0, 0, 0);
        }
        
        // 6. Appliquer les contraintes de la boîte (taille fixe: 1m³)
        PhysicsCalculations::apply_box_constraints(p, BOX_SIZE);
    }
    
    // 7. Gérer les collisions entre particules
    handle_particle_collisions();
    
    // 8. Mettre à jour la visibilité (détection de surface) - pas chaque frame
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

int64_t FluidSimulator::compute_spatial_hash(const Vector3 &position) const {
    // Convertit la position en coordonnées de cellule
    int32_t x = static_cast<int32_t>(std::floor(position.x / spatial_hash_cell_size));
    int32_t y = static_cast<int32_t>(std::floor(position.y / spatial_hash_cell_size));
    int32_t z = static_cast<int32_t>(std::floor(position.z / spatial_hash_cell_size));
    
    // Combine les coordonnées en une seule clé de hash
    // Utilise un grand nombre premier pour éviter les collisions
    const int64_t p1 = 73856093;
    const int64_t p2 = 19349663;
    const int64_t p3 = 83492791;
    
    return (x * p1) ^ (y * p2) ^ (z * p3);
}

void FluidSimulator::rebuild_spatial_hash() {
    // Vide la grille précédente
    spatial_hash.clear();
    
    // Insère chaque particule dans sa cellule
    for (size_t i = 0; i < particles.size(); i++) {
        int64_t hash = compute_spatial_hash(particles[i].position);
        spatial_hash[hash].push_back(i);
    }
}

void FluidSimulator::handle_particle_collisions() {
    // Reconstruit la grille de hachage spatial
    rebuild_spatial_hash();
    
    // Pour chaque cellule de la grille
    for (auto &cell_pair : spatial_hash) {
        std::vector<size_t> &cell_particles = cell_pair.second;
        
        // Collision entre particules de la même cellule
        for (size_t i = 0; i < cell_particles.size(); i++) {
            for (size_t j = i + 1; j < cell_particles.size(); j++) {
                Particle &p1 = particles[cell_particles[i]];
                Particle &p2 = particles[cell_particles[j]];
                
                // Vérifie si collision
                if (PhysicsCalculations::check_particle_collision(p1, p2)) {
                    PhysicsCalculations::resolve_particle_collision(p1, p2);
                }
            }
        }
        
        // Collision avec les particules des cellules adjacentes
        // On vérifie les 26 cellules voisines (3³ - 1)
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    // Skip la cellule courante et la moitié des voisins (pour éviter doublons)
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    if (dx < 0 || (dx == 0 && dy < 0) || (dx == 0 && dy == 0 && dz < 0)) continue;
                    
                    // Calcule le hash de la cellule voisine
                    // On prend la première particule comme référence
                    if (cell_particles.empty()) continue;
                    
                    Vector3 ref_pos = particles[cell_particles[0]].position;
                    Vector3 neighbor_offset(
                        dx * spatial_hash_cell_size,
                        dy * spatial_hash_cell_size,
                        dz * spatial_hash_cell_size
                    );
                    int64_t neighbor_hash = compute_spatial_hash(ref_pos + neighbor_offset);
                    
                    // Vérifie si cette cellule voisine existe
                    auto neighbor_it = spatial_hash.find(neighbor_hash);
                    if (neighbor_it == spatial_hash.end()) continue;
                    
                    std::vector<size_t> &neighbor_particles = neighbor_it->second;
                    
                    // Collision entre cellule courante et cellule voisine
                    for (size_t i : cell_particles) {
                        for (size_t j : neighbor_particles) {
                            Particle &p1 = particles[i];
                            Particle &p2 = particles[j];
                            
                            // Vérifie si collision
                            if (PhysicsCalculations::check_particle_collision(p1, p2)) {
                                PhysicsCalculations::resolve_particle_collision(p1, p2);
                            }
                        }
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

void FluidSimulator::set_velocity_damping(float damping) {
    // Clamp entre 0.0 et 1.0 (0 = arrêt immédiat, 1 = pas de friction)
    velocity_damping = Math::clamp(damping, 0.0f, 1.0f);
}

float FluidSimulator::get_velocity_damping() const {
    return velocity_damping;
}

void FluidSimulator::set_sleep_threshold(float threshold) {
    sleep_threshold = Math::max(threshold, 0.0f);
}

float FluidSimulator::get_sleep_threshold() const {
    return sleep_threshold;
}

void FluidSimulator::set_surface_mesh_enabled(bool enabled) {
    surface_mesh_enabled = enabled;
}

bool FluidSimulator::is_surface_mesh_enabled() const {
    return surface_mesh_enabled;
}

void FluidSimulator::set_surface_threshold(float threshold) {
    surface_threshold = Math::max(threshold, 0.0f);
}

float FluidSimulator::get_surface_threshold() const {
    return surface_threshold;
}

void FluidSimulator::set_surface_grid_resolution(float resolution) {
    surface_grid_resolution = Math::max(resolution, 0.001f); // Minimum 1mm
}

float FluidSimulator::get_surface_grid_resolution() const {
    return surface_grid_resolution;
}

float FluidSimulator::calculate_metaball_density(const Vector3 &position) const {
    float density = 0.0f;
    float influence_radius = reference_particle_radius * 3.0f; // Rayon d'influence des metaballs
    
    // Utilise le spatial hash pour trouver les particules proches efficacement
    int64_t cell_hash = compute_spatial_hash(position);
    
    // Vérifie la cellule actuelle et les cellules voisines
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                Vector3 offset(
                    dx * spatial_hash_cell_size,
                    dy * spatial_hash_cell_size,
                    dz * spatial_hash_cell_size
                );
                int64_t neighbor_hash = compute_spatial_hash(position + offset);
                
                auto it = spatial_hash.find(neighbor_hash);
                if (it == spatial_hash.end()) continue;
                
                const std::vector<size_t> &cell_particles = it->second;
                for (size_t idx : cell_particles) {
                    const Particle &p = particles[idx];
                    float distance = position.distance_to(p.position);
                    
                    // Fonction de densité metaball : f(r) = (1 - (r/R)²)³ si r < R, sinon 0
                    if (distance < influence_radius) {
                        float ratio = distance / influence_radius;
                        float contribution = 1.0f - ratio * ratio;
                        contribution = contribution * contribution * contribution; // ^3
                        density += contribution;
                    }
                }
            }
        }
    }
    
    return density;
}

Array FluidSimulator::generate_surface_mesh() {
    Array result;
    
    if (!surface_mesh_enabled || particles.empty()) {
        return result;
    }
    
    // Reconstruire le spatial hash pour les particules actuelles
    rebuild_spatial_hash();
    
    // Créer une grille pour Marching Cubes
    float cell_size = surface_grid_resolution;
    int grid_x = static_cast<int>(Math::ceil(BOX_SIZE / cell_size));
    int grid_y = static_cast<int>(Math::ceil(BOX_SIZE / cell_size));
    int grid_z = static_cast<int>(Math::ceil(BOX_SIZE / cell_size));
    
    // Arrays pour les vertices et indices du mesh final
    PackedVector3Array vertices;
    PackedInt32Array indices;
    
    // Parcourir la grille et appliquer Marching Cubes sur chaque cellule
    for (int x = 0; x < grid_x - 1; x++) {
        for (int y = 0; y < grid_y - 1; y++) {
            for (int z = 0; z < grid_z - 1; z++) {
                MarchingCubes::GridCell cell;
                
                // Calculer les positions des 8 sommets de la cellule
                // Ordre: (0,0,0), (1,0,0), (1,1,0), (0,1,0), (0,0,1), (1,0,1), (1,1,1), (0,1,1)
                cell.position[0] = Vector3(x * cell_size, y * cell_size, z * cell_size);
                cell.position[1] = Vector3((x+1) * cell_size, y * cell_size, z * cell_size);
                cell.position[2] = Vector3((x+1) * cell_size, (y+1) * cell_size, z * cell_size);
                cell.position[3] = Vector3(x * cell_size, (y+1) * cell_size, z * cell_size);
                cell.position[4] = Vector3(x * cell_size, y * cell_size, (z+1) * cell_size);
                cell.position[5] = Vector3((x+1) * cell_size, y * cell_size, (z+1) * cell_size);
                cell.position[6] = Vector3((x+1) * cell_size, (y+1) * cell_size, (z+1) * cell_size);
                cell.position[7] = Vector3(x * cell_size, (y+1) * cell_size, (z+1) * cell_size);
                
                // Calculer la densité metaball à chaque sommet
                for (int i = 0; i < 8; i++) {
                    cell.value[i] = calculate_metaball_density(cell.position[i]);
                }
                
                // Générer les triangles pour cette cellule
                MarchingCubes::polygonise(cell, surface_threshold, vertices, indices);
            }
        }
    }
    
    // Retourner les données sous forme d'Array pour Godot
    // [0] = vertices (PackedVector3Array), [1] = indices (PackedInt32Array)
    result.resize(2);
    result[0] = vertices;
    result[1] = indices;
    
    UtilityFunctions::print(String("[FluidSimulator] Mesh de surface généré : {0} vertices, {1} indices")
        .format(Array::make(vertices.size(), indices.size())));
    
    return result;
}

bool FluidSimulator::is_particle_on_surface(size_t particle_index) const {
    if (particle_index >= particles.size()) {
        return false;
    }
    
    const Particle &particle = particles[particle_index];
    const String &particle_type = particle.liquid_name;
    
    // Calcule le hash de la cellule où se trouve la particule
    int64_t particle_hash = compute_spatial_hash(particle.position);
    
    // Cherche dans la cellule actuelle
    auto cell_it = spatial_hash.find(particle_hash);
    if (cell_it == spatial_hash.end()) {
        return true; // Pas de voisins, donc visible
    }
    
    // Distance de vérification : un peu plus que 2 rayons (pour toucher les voisins directs)
    float check_distance = particle.radius * 2.2f;
    
    // Directions à vérifier (6 directions principales : haut, bas, gauche, droite, avant, arrière)
    Vector3 directions[6] = {
        Vector3(0, 1, 0),   // Haut
        Vector3(0, -1, 0),  // Bas
        Vector3(1, 0, 0),   // Droite
        Vector3(-1, 0, 0),  // Gauche
        Vector3(0, 0, 1),   // Avant
        Vector3(0, 0, -1)   // Arrière
    };
    
    // Compte combien de directions ont un voisin du même type
    int blocked_directions = 0;
    
    for (int dir = 0; dir < 6; dir++) {
        Vector3 check_pos = particle.position + directions[dir] * check_distance;
        bool has_neighbor_in_direction = false;
        
        // Vérifie dans la cellule actuelle et les cellules voisines
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    Vector3 cell_offset(
                        dx * spatial_hash_cell_size,
                        dy * spatial_hash_cell_size,
                        dz * spatial_hash_cell_size
                    );
                    int64_t neighbor_hash = compute_spatial_hash(particle.position + cell_offset);
                    
                    auto neighbor_cell_it = spatial_hash.find(neighbor_hash);
                    if (neighbor_cell_it == spatial_hash.end()) continue;
                    
                    const std::vector<size_t> &cell_particles = neighbor_cell_it->second;
                    for (size_t neighbor_idx : cell_particles) {
                        if (neighbor_idx == particle_index) continue;
                        
                        const Particle &neighbor = particles[neighbor_idx];
                        
                        // Vérifie si le voisin est du même type
                        if (neighbor.liquid_name != particle_type) continue;
                        
                        // Vérifie si le voisin est dans cette direction
                        Vector3 to_neighbor = neighbor.position - particle.position;
                        float distance = to_neighbor.length();
                        
                        if (distance > 0.001f && distance < check_distance * 1.5f) {
                            to_neighbor = to_neighbor.normalized();
                            float alignment = to_neighbor.dot(directions[dir]);
                            
                            // Si le voisin est dans cette direction (alignment > 0.7 = ~45 degrés)
                            if (alignment > 0.7f) {
                                has_neighbor_in_direction = true;
                                break;
                            }
                        }
                    }
                    if (has_neighbor_in_direction) break;
                }
                if (has_neighbor_in_direction) break;
            }
            if (has_neighbor_in_direction) break;
        }
        
        if (has_neighbor_in_direction) {
            blocked_directions++;
        }
    }
    
    // Si la particule est bloquée dans toutes les directions (ou presque), elle est cachée
    // On laisse une marge : si 5 ou 6 directions sont bloquées, on cache la particule
    if (blocked_directions >= 6) {
        return false; // Cachée (complètement entourée)
    }
    
    return true; // Visible (à la surface)
}

void FluidSimulator::update_particle_visibility() {
    // Reconstruit le hash spatial (déjà fait dans handle_particle_collisions, mais on le refait pour être sûr)
    rebuild_spatial_hash();
    
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
    
    // Paramètres de stabilité
    ClassDB::bind_method(D_METHOD("set_velocity_damping", "damping"), &FluidSimulator::set_velocity_damping);
    ClassDB::bind_method(D_METHOD("get_velocity_damping"), &FluidSimulator::get_velocity_damping);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "velocity_damping", PROPERTY_HINT_RANGE, "0.0,1.0,0.01"), "set_velocity_damping", "get_velocity_damping");
    
    ClassDB::bind_method(D_METHOD("set_sleep_threshold", "threshold"), &FluidSimulator::set_sleep_threshold);
    ClassDB::bind_method(D_METHOD("get_sleep_threshold"), &FluidSimulator::get_sleep_threshold);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "sleep_threshold", PROPERTY_HINT_RANGE, "0.0,0.1,0.0001"), "set_sleep_threshold", "get_sleep_threshold");
    
    // Paramètres de reconstruction de surface
    ClassDB::bind_method(D_METHOD("set_surface_mesh_enabled", "enabled"), &FluidSimulator::set_surface_mesh_enabled);
    ClassDB::bind_method(D_METHOD("is_surface_mesh_enabled"), &FluidSimulator::is_surface_mesh_enabled);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "surface_mesh_enabled"), "set_surface_mesh_enabled", "is_surface_mesh_enabled");
    
    ClassDB::bind_method(D_METHOD("set_surface_threshold", "threshold"), &FluidSimulator::set_surface_threshold);
    ClassDB::bind_method(D_METHOD("get_surface_threshold"), &FluidSimulator::get_surface_threshold);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "surface_threshold", PROPERTY_HINT_RANGE, "0.0,2.0,0.05"), "set_surface_threshold", "get_surface_threshold");
    
    ClassDB::bind_method(D_METHOD("set_surface_grid_resolution", "resolution"), &FluidSimulator::set_surface_grid_resolution);
    ClassDB::bind_method(D_METHOD("get_surface_grid_resolution"), &FluidSimulator::get_surface_grid_resolution);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "surface_grid_resolution", PROPERTY_HINT_RANGE, "0.005,0.1,0.005"), "set_surface_grid_resolution", "get_surface_grid_resolution");
    
    // Génération du mesh de surface
    ClassDB::bind_method(D_METHOD("generate_surface_mesh"), &FluidSimulator::generate_surface_mesh);
}

