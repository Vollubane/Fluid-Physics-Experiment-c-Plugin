#include "particle_physics.hpp"
#include <cmath>

namespace PhysicsCalculations {
    
    void apply_gravity(Particle &particle, const Vector3 &gravity) {
        // F = m * g
        particle.force += particle.mass * gravity;
    }
    
    Vector3 calculate_force_between_particles(const Particle &particle1, const Particle &particle2) {
        // Pour l'instant, retourne un vecteur nul
        // Cette fonction sera utilisée plus tard pour les interactions entre particules
        // (forces de pression, tension de surface, etc.)
        return Vector3(0, 0, 0);
    }
    
    void integrate_euler(Particle &particle, float delta_time) {
        // Méthode d'intégration d'Euler simple
        // a = F / m (accélération = force / masse)
        Vector3 acceleration = particle.force / particle.mass;
        
        // v = v + a * dt
        particle.velocity += acceleration * delta_time;
        
        // p = p + v * dt
        particle.position += particle.velocity * delta_time;
    }
    
    void integrate_verlet(Particle &particle, float delta_time) {
        // Méthode de Verlet: plus stable mais nécessite la position précédente
        // Pour l'instant, on utilise une version simplifiée (Velocity Verlet)
        
        // a = F / m
        Vector3 acceleration = particle.force / particle.mass;
        
        // p = p + v * dt + 0.5 * a * dt²
        particle.position += particle.velocity * delta_time + 0.5f * acceleration * delta_time * delta_time;
        
        // v = v + a * dt
        particle.velocity += acceleration * delta_time;
    }
    
    void reset_forces(Particle &particle) {
        particle.force = Vector3(0, 0, 0);
    }
    
    void apply_box_constraints(Particle &particle, float box_size) {
        // Vérifie chaque axe et applique une contrainte simple
        // La boîte est centrée à l'origine, donc de -box_size/2 à +box_size/2
        float half_box = box_size / 2.0f;
        
        // Contraintes sur X
        if (particle.position.x - particle.radius < -half_box) {
            particle.position.x = -half_box + particle.radius;
            particle.velocity.x = -particle.velocity.x * 0.5f; // Rebond avec perte d'énergie
        } else if (particle.position.x + particle.radius > half_box) {
            particle.position.x = half_box - particle.radius;
            particle.velocity.x = -particle.velocity.x * 0.5f;
        }
        
        // Contraintes sur Y
        if (particle.position.y - particle.radius < -half_box) {
            particle.position.y = -half_box + particle.radius;
            particle.velocity.y = -particle.velocity.y * 0.5f;
        } else if (particle.position.y + particle.radius > half_box) {
            particle.position.y = half_box - particle.radius;
            particle.velocity.y = -particle.velocity.y * 0.5f;
        }
        
        // Contraintes sur Z
        if (particle.position.z - particle.radius < -half_box) {
            particle.position.z = -half_box + particle.radius;
            particle.velocity.z = -particle.velocity.z * 0.5f;
        } else if (particle.position.z + particle.radius > half_box) {
            particle.position.z = half_box - particle.radius;
            particle.velocity.z = -particle.velocity.z * 0.5f;
        }
    }
    
    float calculate_kinetic_energy(const Particle &particle) {
        // E = 0.5 * m * v²
        float velocity_squared = particle.velocity.length_squared();
        return 0.5f * particle.mass * velocity_squared;
    }
    
    float calculate_particle_volume(const Particle &particle) {
        // Volume d'une sphère: V = (4/3) * π * r³
        const float PI = 3.14159265359f;
        return (4.0f / 3.0f) * PI * particle.radius * particle.radius * particle.radius;
    }
    
    float calculate_particle_density(const Particle &particle) {
        // Densité = masse / volume
        float volume = calculate_particle_volume(particle);
        if (volume > 0.0f) {
            return particle.mass / volume;
        }
        return 0.0f;
    }
    
    // ========== Système de collision entre particules ==========
    
    // Vérification AABB rapide (3 comparaisons au lieu d'une racine carrée)
    inline bool aabb_overlap(const Particle &p1, const Particle &p2) {
        float sum_radii = p1.radius + p2.radius;
        return (std::abs(p1.position.x - p2.position.x) < sum_radii &&
                std::abs(p1.position.y - p2.position.y) < sum_radii &&
                std::abs(p1.position.z - p2.position.z) < sum_radii);
    }
    
    bool check_particle_collision(const Particle &p1, const Particle &p2) {
        // Test AABB rapide d'abord (3 comparaisons)
        if (!aabb_overlap(p1, p2)) {
            return false;
        }
        
        // Test sphérique précis seulement si AABB overlap
        Vector3 delta = p1.position - p2.position;
        float distance_squared = delta.length_squared();
        
        // Somme des rayons
        float radius_sum = p1.radius + p2.radius;
        float radius_sum_squared = radius_sum * radius_sum;
        
        // Collision si la distance est inférieure à la somme des rayons
        return distance_squared < radius_sum_squared;
    }
    
    void separate_overlapping_particles(Particle &p1, Particle &p2) {
        // Vecteur de p2 vers p1
        Vector3 delta = p1.position - p2.position;
        float distance = delta.length();
        
        // Si les particules sont exactement au même endroit, on applique une petite séparation aléatoire
        if (distance < 0.0001f) {
            delta = Vector3(0.01f, 0.0f, 0.0f);
            distance = 0.01f;
        }
        
        // Calcul du chevauchement
        float overlap = (p1.radius + p2.radius) - distance;
        
        // Si pas de chevauchement, ne rien faire
        if (overlap <= 0.0f) {
            return;
        }
        
        // Direction normalisée de la séparation
        Vector3 separation_direction = delta / distance;
        
        // Déplacement proportionnel aux masses (les particules plus lourdes bougent moins)
        float total_mass = p1.mass + p2.mass;
        float ratio1 = p2.mass / total_mass; // p1 se déplace proportionnellement à la masse de p2
        float ratio2 = p1.mass / total_mass; // p2 se déplace proportionnellement à la masse de p1
        
        // LIMITER la séparation pour éviter les mouvements explosifs
        // Si chevauchement > rayon moyen, limiter à 50% du rayon moyen par itération
        float max_separation = (p1.radius + p2.radius) * 0.5f;
        float separation_amount = overlap;
        if (separation_amount > max_separation) {
            separation_amount = max_separation;
        }
        
        // Applique la séparation limitée
        p1.position += separation_direction * separation_amount * ratio1;
        p2.position -= separation_direction * separation_amount * ratio2;
    }
    
    void resolve_particle_collision(Particle &p1, Particle &p2) {
        // Vecteur entre les deux particules
        Vector3 delta = p1.position - p2.position;
        float distance = delta.length();
        
        // Évite la division par zéro
        if (distance < 0.0001f) {
            return;
        }
        
        // Direction de collision normalisée
        Vector3 collision_normal = delta / distance;
        
        // Vitesse relative
        Vector3 relative_velocity = p1.velocity - p2.velocity;
        
        // Vitesse relative le long de la normale de collision
        float velocity_along_normal = relative_velocity.dot(collision_normal);
        
        // Ne résout la collision que si les particules se rapprochent
        if (velocity_along_normal > 0.0f) {
            return; // Les particules s'éloignent déjà
        }
        
        // Coefficient de restitution moyen des deux particules
        float restitution = (p1.restitution + p2.restitution) * 0.5f;
        
        // Calcule l'impulsion de collision
        // Formule: j = -(1 + e) * v_rel · n / (1/m1 + 1/m2)
        // où e est le coefficient de restitution
        float impulse_magnitude = -(1.0f + restitution) * velocity_along_normal;
        impulse_magnitude /= (1.0f / p1.mass + 1.0f / p2.mass);
        
        // Vecteur d'impulsion
        Vector3 impulse = collision_normal * impulse_magnitude;
        
        // Applique l'impulsion aux vélocités (conservation de la quantité de mouvement)
        p1.velocity += impulse / p1.mass;
        p2.velocity -= impulse / p2.mass;
        
        // Sépare les particules qui se chevauchent
        separate_overlapping_particles(p1, p2);
    }
    
    // ========== Double Buffering pour collisions parallèles ==========
    
    // Calcule la collision entre deux particules et retourne les deltas à appliquer
    // Version LOCK-FREE: lit les données actuelles, retourne les corrections
    void calculate_collision_to_buffer(const Particle &p1_read, const Particle &p2_read,
                                       Vector3 &p1_velocity_delta, Vector3 &p2_velocity_delta,
                                       Vector3 &p1_position_delta, Vector3 &p2_position_delta) {
        // Réinitialiser les deltas
        p1_velocity_delta = Vector3(0, 0, 0);
        p2_velocity_delta = Vector3(0, 0, 0);
        p1_position_delta = Vector3(0, 0, 0);
        p2_position_delta = Vector3(0, 0, 0);
        
        // Vecteur entre les centres des particules
        Vector3 delta = p1_read.position - p2_read.position;
        float distance_squared = delta.length_squared();
        
        // Rayon combiné
        float combined_radius = p1_read.radius + p2_read.radius;
        float combined_radius_squared = combined_radius * combined_radius;
        
        // Pas de collision si distance > somme des rayons
        if (distance_squared >= combined_radius_squared) {
            return;
        }
        
        // Distance réelle
        float distance = std::sqrt(distance_squared);
        
        // Éviter division par zéro (particules exactement au même endroit)
        if (distance < 1e-6f) {
            distance = 1e-6f;
            delta = Vector3(1, 0, 0) * distance; // Direction arbitraire
        }
        
        // Direction de collision normalisée
        Vector3 collision_normal = delta / distance;
        
        // ========== CALCUL DE VÉLOCITÉ ==========
        
        // Vitesse relative
        Vector3 relative_velocity = p1_read.velocity - p2_read.velocity;
        
        // Vitesse relative le long de la normale
        float velocity_along_normal = relative_velocity.dot(collision_normal);
        
        // Ne résout que si les particules se rapprochent
        if (velocity_along_normal > 0.0f) {
            return; // S'éloignent déjà
        }
        
        // Coefficient de restitution moyen
        float restitution = (p1_read.restitution + p2_read.restitution) * 0.5f;
        
        // Impulsion de collision
        float impulse_magnitude = -(1.0f + restitution) * velocity_along_normal;
        impulse_magnitude /= (1.0f / p1_read.mass + 1.0f / p2_read.mass);
        
        // Vecteur d'impulsion
        Vector3 impulse = collision_normal * impulse_magnitude;
        
        // Calcul des deltas de vélocité
        p1_velocity_delta = impulse / p1_read.mass;
        p2_velocity_delta = -impulse / p2_read.mass;
        
        // ========== CALCUL DE SÉPARATION ==========
        
        // Chevauchement
        float overlap = combined_radius - distance;
        
        if (overlap > 0.0f) {
            // Proportion de masse pour répartir la séparation
            float total_mass = p1_read.mass + p2_read.mass;
            float p1_ratio = p2_read.mass / total_mass; // Particule lourde bouge moins
            float p2_ratio = p1_read.mass / total_mass;
            
            // Calcul des deltas de position (séparation)
            p1_position_delta = collision_normal * (overlap * p1_ratio);
            p2_position_delta = -collision_normal * (overlap * p2_ratio);
        }
    }
    
    // ========== Viscosité et liens entre particules ==========
    
    // Vérifie si deux particules sont assez proches pour être connectées
    bool check_viscosity_connection(const Particle &p1, const Particle &p2, float connection_radius) {
        // Vérifie d'abord si elles sont du même liquide
        if (p1.liquid_name != p2.liquid_name) {
            return false;
        }
        
        // Calcule la distance entre les centres
        Vector3 delta = p1.position - p2.position;
        float distance_squared = delta.length_squared();
        
        // Rayon de connexion = moyenne des rayons × multiplicateur
        float avg_radius = (p1.radius + p2.radius) * 0.5f;
        float connection_dist = avg_radius * connection_radius;
        float connection_dist_squared = connection_dist * connection_dist;
        
        return distance_squared <= connection_dist_squared;
    }
    
    // Applique une force de viscosité basée sur les voisins
    void apply_viscosity_force(Particle &particle, const std::vector<Particle> &all_particles, float viscosity) {
        if (viscosity <= 0.0f || particle.neighbors.empty()) {
            return; // Pas de viscosité ou pas de voisins
        }
        
        // Force de viscosité = somme des différences de vélocité avec les voisins
        Vector3 viscosity_force(0, 0, 0);
        int valid_neighbors = 0;
        
        for (size_t neighbor_idx : particle.neighbors) {
            if (neighbor_idx >= all_particles.size()) continue;
            
            const Particle &neighbor = all_particles[neighbor_idx];
            
            // Vérifie si toujours du même liquide (peut changer avec le temps)
            if (neighbor.liquid_name != particle.liquid_name) continue;
            
            // Différence de vélocité
            Vector3 velocity_diff = neighbor.velocity - particle.velocity;
            
            // Force de viscosité proportionnelle à la différence de vélocité
            viscosity_force += velocity_diff * viscosity;
            valid_neighbors++;
        }
        
        // Moyenne de la force (pour éviter que trop de voisins = force énorme)
        if (valid_neighbors > 0) {
            viscosity_force /= static_cast<float>(valid_neighbors);
            
            // Applique la force (multiplication par la masse pour respecter F=ma)
            particle.force += viscosity_force * particle.mass;
        }
    }
    
    // ========== Prédiction d'oscillation ==========
    
    // Simule N frames en avance sans modifier la particule réelle
    // Retourne la norme de la somme vectorielle de tous les déplacements
    // Si proche de 0 → la particule oscille (va dans un sens puis revient)
    float predict_movement_sum(const Particle &particle, const Vector3 &gravity, 
                              float delta_time, int lookahead_frames, 
                              float damping, float box_size) {
        // Crée une copie temporaire pour la simulation prédictive
        Particle temp = particle;
        
        // Somme vectorielle de tous les déplacements futurs
        Vector3 total_displacement(0, 0, 0);
        
        // Simule N frames en avance
        for (int frame = 0; frame < lookahead_frames; frame++) {
            Vector3 position_before = temp.position;
            
            // 1. Réinitialiser les forces
            reset_forces(temp);
            
            // 2. Appliquer la gravité
            apply_gravity(temp, gravity);
            
            // 3. Intégrer (calculer nouvelle position)
            integrate_euler(temp, delta_time);
            
            // 4. Appliquer damping
            temp.velocity *= damping;
            
            // 5. Contraintes de boîte
            apply_box_constraints(temp, box_size);
            
            // 6. Calculer le déplacement de cette frame
            Vector3 displacement = temp.position - position_before;
            
            // 7. Ajouter à la somme vectorielle
            total_displacement += displacement;
        }
        
        // Retourne la norme de la somme totale
        // Si cette valeur est proche de 0 → oscillation
        return total_displacement.length();
    }
}

