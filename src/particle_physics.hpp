#pragma once

#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/color.hpp>

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
          visible(true) {}
    
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
          visible(true) {}
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
}

