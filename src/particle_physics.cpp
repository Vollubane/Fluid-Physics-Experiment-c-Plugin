#include "particle_physics.hpp"

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
    
    bool check_particle_collision(const Particle &p1, const Particle &p2) {
        // Calcule la distance entre les centres des deux particules
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
        
        // Applique la séparation complète
        p1.position += separation_direction * overlap * ratio1;
        p2.position -= separation_direction * overlap * ratio2;
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
}

