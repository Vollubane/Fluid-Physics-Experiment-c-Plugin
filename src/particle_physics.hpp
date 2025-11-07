#pragma once

#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/color.hpp>
#include <vector>

using namespace godot;

// Structure représentant une particule de fluide
// Cette classe pure C++ n'est pas exposée à Godot
struct Particle {
    Vector3 position;    // Position dans l'espace 3D (en mètres)
    Vector3 velocity;    // Vélocité de la particule (m/s)
    Vector3 force;       // Force accumulée appliquée sur la particule (Newton)
    
    float mass;          // Masse de la particule (kg)
    float radius;        // Rayon de la particule (m) - utilisé pour la taille
    float density;       // Densité de cette particule (kg/m³)
    float restitution;   // Coefficient de restitution de cette particule (0.0 à 1.0)
    
    String liquid_name;  // Nom du liquide (ex: "eau", "huile", "pétrole")
    Color color;         // Couleur de la particule (pour colorant/diffusion)
    bool visible;        // Est visible si en surface (pas complètement entourée)
    
    // Viscosité et optimisation
    std::vector<size_t> neighbors;  // Indices des voisins du même liquide (pour viscosité)
    int connection_count;            // Nombre de connexions (mis à jour chaque frame)
    bool is_interior;                // True si particule au centre (beaucoup de connexions)
    
    // Constructeur par défaut (eau bleue)
    Particle() 
        : position(Vector3(0, 0, 0)), 
          velocity(Vector3(0, 0, 0)), 
          force(Vector3(0, 0, 0)),
          mass(1.0f), 
          radius(0.05f),
          density(1000.0f),
          restitution(0.0f),  // Aucun rebond - collision parfaitement inélastique
          liquid_name("eau"),
          color(Color(0.3f, 0.6f, 1.0f, 1.0f)),
          visible(true),
          connection_count(0),
          is_interior(false) {}
    
    // Constructeur avec paramètres complets
    Particle(Vector3 pos, float m, float r, float dens, float rest, const String &name, const Color &col)
        : position(pos),
          velocity(Vector3(0, 0, 0)),
          force(Vector3(0, 0, 0)),
          mass(m),
          radius(r),
          density(dens),
          restitution(rest),
          liquid_name(name),
          color(col),
          visible(true),
          connection_count(0),
          is_interior(false) {}
};

// Namespace contenant toutes les fonctions de calcul physique
namespace PhysicsCalculations {
    
    // Applique la gravité à une particule
    // gravity: vecteur de gravité (par défaut: Vector3(0, -9.81, 0))
    void apply_gravity(Particle &particle, const Vector3 &gravity);
    
    // Calcule la force entre deux particules (pour interactions futures)
    // Retourne la force appliquée sur particle1 par particle2
    Vector3 calculate_force_between_particles(const Particle &particle1, const Particle &particle2);
    
    // Intégration de la vélocité et position avec méthode d'Euler
    // delta_time: temps écoulé depuis la dernière frame (secondes)
    void integrate_euler(Particle &particle, float delta_time);
    
    // Intégration avec méthode de Verlet (plus stable pour la physique)
    void integrate_verlet(Particle &particle, float delta_time);
    
    // Réinitialise les forces accumulées d'une particule
    void reset_forces(Particle &particle);
    
    // Applique les contraintes de la boîte cubique (1m³)
    // Empêche les particules de sortir du cube
    void apply_box_constraints(Particle &particle, float box_size);
    
    // Calcule l'énergie cinétique d'une particule
    float calculate_kinetic_energy(const Particle &particle);
    
    // Calcule le volume d'une particule sphérique
    float calculate_particle_volume(const Particle &particle);
    
    // Calcule la densité d'une particule (masse/volume)
    float calculate_particle_density(const Particle &particle);
    
    // ========== Système de collision entre particules ==========
    
    // Vérifie si deux particules sont en collision
    // Retourne true si la distance entre les centres est inférieure à la somme des rayons
    bool check_particle_collision(const Particle &p1, const Particle &p2);
    
    // Résout la collision entre deux particules
    // Utilise le coefficient de restitution moyen des deux particules
    void resolve_particle_collision(Particle &p1, Particle &p2);
    
    // Sépare deux particules qui se chevauchent
    // Déplace les particules pour qu'elles se touchent juste sans se chevaucher
    void separate_overlapping_particles(Particle &p1, Particle &p2);
    
    // ========== Double Buffering pour collisions parallèles ==========
    
    // Calcule la collision entre deux particules et écrit dans les buffers temporaires
    // Version lock-free: lit position/velocity, écrit temp_position/temp_velocity
    // Accumule les corrections de vélocité (peut être appelé plusieurs fois)
    void calculate_collision_to_buffer(const Particle &p1_read, const Particle &p2_read,
                                       Vector3 &p1_velocity_delta, Vector3 &p2_velocity_delta,
                                       Vector3 &p1_position_delta, Vector3 &p2_position_delta);
    
    // ========== Viscosité et liens entre particules ==========
    
    // Applique une force de viscosité basée sur les voisins du même liquide
    // viscosity: coefficient de viscosité (0 = aucune viscosité, 1 = très visqueux)
    void apply_viscosity_force(Particle &particle, const std::vector<Particle> &all_particles, float viscosity);
    
    // Vérifie si deux particules sont assez proches pour être connectées (viscosité)
    // connection_radius: rayon de connexion (multiplicateur du rayon des particules)
    bool check_viscosity_connection(const Particle &p1, const Particle &p2, float connection_radius);
    
    // ========== Prédiction d'oscillation ==========
    
    // Simule N frames en avance et calcule la somme des déplacements
    // Retourne la norme de la somme vectorielle des déplacements
    // Si proche de 0 → oscillation détectée
    float predict_movement_sum(const Particle &particle, const Vector3 &gravity, 
                              float delta_time, int lookahead_frames, 
                              float damping, float box_size);
}

