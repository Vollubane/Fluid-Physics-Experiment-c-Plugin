#pragma once

#include "particle_physics.hpp"
#include <godot_cpp/classes/node3d.hpp>
#include <vector>
#include <unordered_map>

using namespace godot;

// Classe principale du simulateur de fluides
// Cette classe est exposée à Godot et peut être ajoutée à la scène
class FluidSimulator : public Node3D {
    GDCLASS(FluidSimulator, Node3D)

protected:
    static void _bind_methods();

private:
    // Conteneur de toutes les particules
    std::vector<Particle> particles;
    
    // Paramètres de simulation
    static constexpr float BOX_SIZE = 1.0f;  // Taille fixe de la boîte : 1m³
    static constexpr float BOX_VOLUME = 1.0f; // Volume en m³ (donc 1000 litres)
    static constexpr float POUR_DISC_RADIUS = 0.05f; // Rayon du disque de versement : 5cm
    
    Vector3 gravity;             // Vecteur de gravité
    bool simulation_active;      // État de la simulation
    
    // Paramètres de référence (pour info seulement, non utilisés pour le calcul)
    int max_particles;           // Nombre maximum de particules (pour calcul de référence)
    
    // Hachage spatial pour optimisation des collisions
    float spatial_hash_cell_size; // Taille d'une cellule de la grille spatiale
    std::unordered_map<int64_t, std::vector<size_t>> spatial_hash; // Hash -> indices des particules
    
    // Valeurs de référence (calculées pour max_particles)
    float reference_particle_volume;  // Volume de référence d'une particule (m³)
    float reference_particle_radius;  // Rayon de référence d'une particule (m)
    
    // Système de versement progressif
    bool is_pouring;             // Versement en cours ?
    float pour_timer;            // Timer actuel du versement
    float pour_duration;         // Durée totale du versement (secondes)
    int pour_particles_total;    // Nombre total de particules à verser
    int pour_particles_added;    // Nombre de particules déjà ajoutées
    float pour_rate;             // Particules par seconde
    int pour_pattern_index;      // Index pour la distribution homogène en spirale
    
    // Paramètres du liquide en cours de versement
    String pour_liquid_name;     // Nom du liquide à verser
    Color pour_color;            // Couleur du liquide à verser
    float pour_density;          // Densité du liquide à verser (kg/m³)
    float pour_restitution;      // Restitution du liquide à verser
    float pour_particle_volume;  // Volume d'une particule de ce liquide
    float pour_particle_radius;  // Rayon d'une particule de ce liquide
    float pour_particle_mass;    // Masse d'une particule de ce liquide
    
    // Paramètres de visibilité
    int visibility_check_frequency; // Vérifier la visibilité toutes les N frames
    int current_frame;              // Compteur de frames
    
    // Paramètres de stabilité
    float velocity_damping;         // Coefficient d'amortissement de la vitesse (0.0 à 1.0)
    float sleep_threshold;          // Seuil de vitesse pour mettre au repos (m/s)
    
    // Paramètres de reconstruction de surface (Marching Cubes)
    bool surface_mesh_enabled;      // Activer la génération de mesh de surface
    float surface_threshold;        // Seuil de densité pour la surface (metaballs)
    float surface_grid_resolution;  // Taille des cellules de la grille (m)

private:
    // Recalcule les propriétés de référence selon max_particles
    void recalculate_reference_properties();
    
    // Calcule la densité metaball en un point (pour reconstruction de surface)
    float calculate_metaball_density(const Vector3 &position) const;
    
    // Ajoute une particule dans le disque de versement (au centre haut du cube)
    void add_particle_in_pour_disc();
    
    // Met à jour le système de versement progressif
    void update_pouring(float delta_time);
    
    // ========== Hachage spatial ==========
    
    // Calcule la clé de hash pour une position donnée
    int64_t compute_spatial_hash(const Vector3 &position) const;
    
    // Reconstruit la grille de hachage spatial
    void rebuild_spatial_hash();
    
    // Gère toutes les collisions entre particules (avec hachage spatial)
    void handle_particle_collisions();
    
    // Met à jour la visibilité des particules (détection de surface)
    void update_particle_visibility();
    
    // Vérifie si une particule est à la surface (a au moins un voisin différent ou aucun voisin)
    bool is_particle_on_surface(size_t particle_index) const;

public:
    // Constructeur
    FluidSimulator();
    
    // Destructeur
    ~FluidSimulator();
    
    // ========== Fonctions de gestion des particules ==========
    
    // Démarre le versement progressif de liquide depuis le haut du cube
    // liquid_name: nom du liquide (ex: "eau", "huile", "miel")
    // color: couleur du liquide (pour colorant/visualisation)
    // density: densité du liquide en kg/m³
    // restitution: coefficient de restitution (0.0 à 1.0)
    // liters: quantité de liquide à verser en litres
    // duration: temps pour verser en secondes
    void start_pouring_liquid(const String &liquid_name, const Color &color, float density, float restitution, float liters, float duration);
    
    // Arrête le versement en cours
    void stop_pouring();
    
    // Vérifie si un versement est en cours
    bool is_pouring_active() const;
    
    // Obtient le pourcentage de versement (0.0 à 1.0)
    float get_pouring_progress() const;
    
    // Retire toutes les particules
    void clear_particles();
    
    // Retourne le nombre de particules actuelles
    int get_particle_count() const;
    
    // ========== Fonctions de simulation ==========
    
    // Met à jour la simulation (appelé chaque frame)
    void update_simulation(float delta_time);
    
    // Active/désactive la simulation
    void set_simulation_active(bool active);
    bool is_simulation_active() const;
    
    // ========== Getters et Setters ==========
    
    // Taille de la boîte (lecture seule, toujours 1m³)
    float get_box_size() const;
    
    // Volume de la boîte en litres (lecture seule, toujours 1000L)
    float get_box_volume_liters() const;
    
    // Gravité
    void set_gravity(Vector3 g);
    Vector3 get_gravity() const;
    
    // Nombre maximum de particules (recalcule automatiquement les propriétés)
    void set_max_particles(int count);
    int get_max_particles() const;
    
    // Propriétés de référence calculées (lecture seule)
    float get_reference_particle_volume() const;
    float get_reference_particle_radius() const;
    
    // ========== Fonctions d'information ==========
    
    // Obtient la position d'une particule spécifique
    Vector3 get_particle_position(int index) const;
    
    // Obtient la vélocité d'une particule spécifique
    Vector3 get_particle_velocity(int index) const;
    
    // Obtient le nom du liquide d'une particule spécifique
    String get_particle_liquid_name(int index) const;
    
    // Obtient la densité d'une particule spécifique
    float get_particle_density(int index) const;
    
    // Obtient la couleur d'une particule spécifique
    Color get_particle_color(int index) const;
    
    // Vérifie si une particule est visible
    bool is_particle_visible(int index) const;
    
    // Compte le nombre de particules visibles
    int get_visible_particle_count() const;
    
    // Calcule l'énergie totale du système
    float get_total_kinetic_energy() const;
    
    // Paramètres de visibilité
    void set_visibility_check_frequency(int frequency);
    int get_visibility_check_frequency() const;
    
    // Paramètres de stabilité
    void set_velocity_damping(float damping);
    float get_velocity_damping() const;
    void set_sleep_threshold(float threshold);
    float get_sleep_threshold() const;
    
    // Paramètres de reconstruction de surface
    void set_surface_mesh_enabled(bool enabled);
    bool is_surface_mesh_enabled() const;
    void set_surface_threshold(float threshold);
    float get_surface_threshold() const;
    void set_surface_grid_resolution(float resolution);
    float get_surface_grid_resolution() const;
    
    // Génération du mesh de surface (Marching Cubes)
    Array generate_surface_mesh();
    
    // ========== Fonctions Godot ==========
    
    // Appelé quand le node entre dans la scène
    void _ready();
    
    // Appelé chaque frame physique
    void _physics_process(double delta);
};

