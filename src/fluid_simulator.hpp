#pragma once

#include "particle_physics.hpp"
#include <godot_cpp/classes/node3d.hpp>
#include <vector>
#include <unordered_map>
#include <cfloat>  // Pour FLT_MAX
#include <thread>  // Pour std::thread
#include <mutex>   // Pour std::mutex
#include <atomic>  // Pour std::atomic

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
    
    // Grille spatiale uniforme 3D pour optimisation des collisions (O(1) au lieu de O(log n))
    std::vector<std::vector<size_t>> spatial_grid;  // Grille pré-allouée
    int grid_resolution;        // Nombre de cellules par dimension (20 = grille 20x20x20)
    float cell_size;            // Taille d'une cellule (en mètres)
    
    // Buffers temporaires pour le double buffering des collisions (lock-free threading)
    std::vector<Vector3> velocity_deltas;  // Accumulation des corrections de vélocité
    std::vector<Vector3> position_deltas;  // Accumulation des corrections de position
    
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
    
    // Sous-étapes de simulation (substeps) - seule vraie solution pour stabilité
    int substeps;                   // Nombre de sous-étapes par frame (défaut: 10)
    
    // Viscosité et optimisation des particules intérieures
    float viscosity;                // Coefficient de viscosité global (0-100, défaut: 5.0)
    float connection_radius;        // Rayon de connexion (multiplicateur, défaut: 2.5)
    int interior_threshold;         // Nombre de connexions pour être "intérieur" (défaut: 18)
    bool optimize_interior;         // Réduire les calculs pour particules intérieures (défaut: true)
    
    // Caméra pour occlusion culling 2D (méthode software rasterizer)
    Node3D* camera;                 // Pointeur vers la caméra active (nullptr = afficher tout)
    
    // Parallélisation de la physique
    int num_threads;                // Nombre de threads (0 = auto-detect)
    bool use_multithreading;        // Activer le multi-threading (défaut: true si > 1000 particules)

private:
    // ========== Structure pour occlusion culling 2D (software rasterizer) ==========
    struct ScreenPixel {
        size_t particle_index;  // Index de la particule la plus proche
        float depth;            // Profondeur (distance à la caméra)
        
        ScreenPixel() : particle_index(SIZE_MAX), depth(FLT_MAX) {}
    };
    
    // Résolution du buffer d'occlusion (plus c'est grand, plus c'est précis mais lent)
    int occlusion_buffer_width;   // Largeur du buffer (défaut: 128)
    int occlusion_buffer_height;  // Hauteur du buffer (défaut: 128)
    mutable std::vector<ScreenPixel> occlusion_buffer;  // Buffer 2D aplati
    mutable bool occlusion_valid;  // Le buffer est-il à jour ?
    
    // Recalcule les propriétés de référence selon max_particles
    void recalculate_reference_properties();
    
    // Ajoute une particule dans le disque de versement (au centre haut du cube)
    void add_particle_in_pour_disc();
    
    // Met à jour le système de versement progressif
    void update_pouring(float delta_time);
    
    // ========== Grille spatiale uniforme ==========
    
    // Calcule l'index de cellule pour une position (inline pour performance)
    inline int get_cell_index(const Vector3 &position) const {
        int x = static_cast<int>(position.x / cell_size);
        int y = static_cast<int>(position.y / cell_size);
        int z = static_cast<int>(position.z / cell_size);
        
        // Clamper aux limites de la grille
        x = (x < 0) ? 0 : (x >= grid_resolution) ? grid_resolution - 1 : x;
        y = (y < 0) ? 0 : (y >= grid_resolution) ? grid_resolution - 1 : y;
        z = (z < 0) ? 0 : (z >= grid_resolution) ? grid_resolution - 1 : z;
        
        return x + y * grid_resolution + z * grid_resolution * grid_resolution;
    }
    
    // Reconstruit la grille spatiale (clear + remplissage, pas de réallocation)
    void rebuild_spatial_grid();
    
    // Gère toutes les collisions entre particules
    void handle_particle_collisions();
    
    // ========== Parallélisation de la physique ==========
    
    // Applique les forces et intègre la physique pour un range de particules (thread-safe)
    void process_physics_range(size_t start, size_t end, float sub_dt);
    
    // Traite les collisions pour une partition de la grille spatiale (thread-safe)
    void process_collisions_partition(int start_cell, int end_cell);  // VERSION ANCIENNE
    
    // ========== Double Buffering pour collisions (100% lock-free) ==========
    
    // Phase 1: Calcule les collisions et stocke dans velocity_deltas/position_deltas (parallèle)
    void process_collisions_calculate(int start_cell, int end_cell);
    
    // Phase 2: Applique les deltas aux particules (parallèle)
    void process_collisions_apply(size_t start, size_t end);
    
    // ========== Viscosité et optimisation ==========
    
    // Reconstruit les liens de voisinage entre particules du même liquide
    void rebuild_viscosity_connections();
    
    // Applique les forces de viscosité à toutes les particules
    void apply_viscosity_forces();
    
    // Détermine quelles particules sont à l'intérieur (beaucoup de connexions)
    void update_interior_particles();
    
    // Met à jour la visibilité des particules (occlusion culling 2D)
    void update_particle_visibility();
    
    // ========== Occlusion Culling 2D (Software Rasterizer) ==========
    
    // Projette une position 3D vers les coordonnées écran 2D
    bool project_to_screen(const Vector3& world_pos, const Transform3D& view_matrix, 
                           const Transform3D& projection_matrix, 
                           int& screen_x, int& screen_y, float& depth) const;
    
    // Rasterise un disque (particule) sur le buffer d'occlusion
    void rasterize_particle_disk(size_t particle_index, int center_x, int center_y, 
                                   int radius_pixels, float depth);
    
    // Rasterise un disque dans un buffer spécifique (thread-safe)
    void rasterize_particle_disk_to_buffer(std::vector<ScreenPixel>& buffer, 
                                             size_t particle_index, int center_x, int center_y, 
                                             int radius_pixels, float depth) const;
    
    // Reconstruit le buffer d'occlusion à partir de toutes les particules
    void rebuild_occlusion_buffer() const;
    
    // Rasterise un range de particules dans un buffer temporaire (thread-safe)
    void rasterize_particles_range(size_t start, size_t end, 
                                    const Transform3D& view_matrix,
                                    const Transform3D& projection_matrix,
                                    float particle_radius_world, float f,
                                    std::vector<ScreenPixel>& temp_buffer) const;
    
    // Fusionne un buffer temporaire dans le buffer principal (avec Z-test)
    void merge_buffer(const std::vector<ScreenPixel>& temp_buffer) const;
    
    // Vérifie si une particule est visible (présente dans le buffer d'occlusion)
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
    
    // Paramètres de substeps
    void set_substeps(int steps);
    int get_substeps() const;
    
    // Paramètres de viscosité
    void set_viscosity(float visc);
    float get_viscosity() const;
    void set_connection_radius(float radius);
    float get_connection_radius() const;
    void set_interior_threshold(int threshold);
    int get_interior_threshold() const;
    void set_optimize_interior(bool enable);
    bool get_optimize_interior() const;
    
    // Caméra pour occlusion culling 2D (passer une Camera3D depuis GDScript)
    void set_camera(Node3D* cam);
    Node3D* get_camera() const;
    
    // Résolution du buffer d'occlusion (défaut: 128x128)
    void set_occlusion_buffer_resolution(int width, int height);
    int get_occlusion_buffer_width() const;
    int get_occlusion_buffer_height() const;
    
    // Parallélisation de la physique
    void set_num_threads(int threads);  // 0 = auto-detect, -1 = désactiver
    int get_num_threads() const;
    void set_use_multithreading(bool enable);
    bool get_use_multithreading() const;
    
    // ========== Fonctions Godot ==========
    
    // Appelé quand le node entre dans la scène
    void _ready();
    
    // Appelé chaque frame physique
    void _physics_process(double delta);
};

