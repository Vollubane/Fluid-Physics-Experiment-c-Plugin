extends Node3D

# Exemple de mélange de plusieurs liquides avec couleurs et détection de surface

@onready var fluid_sim = $FluidSimulator

func _ready():
    # Configuration de base
    fluid_sim.max_particles = 10000
    fluid_sim.visibility_check_frequency = 10  # Vérifier visibilité toutes les 10 frames
    
    print("=== Simulateur de fluides avec détection de surface ===")
    print("Volume du conteneur: ", fluid_sim.get_box_volume_liters(), " litres")
    print("")
    
    # Verser de l'eau (bleue)
    print("1. Versement d'eau bleue...")
    fluid_sim.start_pouring_liquid("eau", Color(0.3, 0.6, 1.0, 1.0), 1000.0, 0.4, 100.0, 3.0)
    
    # Après 4 secondes, verser de l'huile (jaune, flottera)
    await get_tree().create_timer(4.0).timeout
    print("\n2. Versement d'huile jaune (devrait flotter)...")
    fluid_sim.start_pouring_liquid("huile", Color(1.0, 0.9, 0.2, 1.0), 920.0, 0.3, 50.0, 2.0)
    
    # Après 3 secondes, verser du colorant rouge dans l'eau
    await get_tree().create_timer(3.0).timeout
    print("\n3. Versement de colorant rouge...")
    fluid_sim.start_pouring_liquid("eau_rouge", Color(1.0, 0.2, 0.2, 1.0), 1000.0, 0.4, 20.0, 2.0)
    
    # Après 3 secondes, verser du miel (orange)
    await get_tree().create_timer(3.0).timeout
    print("\n4. Versement de miel orange...")
    fluid_sim.start_pouring_liquid("miel", Color(1.0, 0.6, 0.1, 1.0), 1420.0, 0.2, 30.0, 3.0)

func _process(delta):
    # Affiche les statistiques toutes les secondes
    if Engine.get_frames_drawn() % 60 == 0:
        var total = fluid_sim.get_particle_count()
        var visible = fluid_sim.get_visible_particle_count()
        var hidden = total - visible
        var percentage_visible = (float(visible) / float(total) * 100.0) if total > 0 else 0.0
        
        print("Particules: %d total | %d visibles (%.1f%%) | %d cachées" % [total, visible, percentage_visible, hidden])
        print("  Énergie: %.2f J" % fluid_sim.get_total_kinetic_energy())
        
        # Affiche un échantillon de particules
        if total > 0:
            var sample_index = min(total - 1, 10)
            var liquid_name = fluid_sim.get_particle_liquid_name(sample_index)
            var density = fluid_sim.get_particle_density(sample_index)
            var color = fluid_sim.get_particle_color(sample_index)
            var visible_flag = fluid_sim.is_particle_visible(sample_index)
            print("  Particule #%d: %s (%.0f kg/m³) - couleur: (%d,%d,%d) - %s" % [
                sample_index, liquid_name, density, 
                int(color.r * 255), int(color.g * 255), int(color.b * 255),
                "visible" if visible_flag else "cachée"
            ])

func _input(event):
    if event is InputEventKey and event.pressed:
        match event.keycode:
            KEY_1:
                # Verser de l'eau bleue
                print("\n[Touche 1] Versement d'eau bleue...")
                fluid_sim.start_pouring_liquid("eau", Color(0.3, 0.6, 1.0), 1000.0, 0.4, 50.0, 2.0)
            
            KEY_2:
                # Verser de l'huile jaune
                print("\n[Touche 2] Versement d'huile jaune...")
                fluid_sim.start_pouring_liquid("huile", Color(1.0, 0.9, 0.2), 920.0, 0.3, 50.0, 2.0)
            
            KEY_3:
                # Verser du colorant vert
                print("\n[Touche 3] Versement de colorant vert...")
                fluid_sim.start_pouring_liquid("colorant_vert", Color(0.2, 1.0, 0.3), 1000.0, 0.4, 20.0, 1.5)
            
            KEY_4:
                # Verser du colorant rouge
                print("\n[Touche 4] Versement de colorant rouge...")
                fluid_sim.start_pouring_liquid("colorant_rouge", Color(1.0, 0.2, 0.2), 1000.0, 0.4, 20.0, 1.5)
            
            KEY_5:
                # Verser du miel orange
                print("\n[Touche 5] Versement de miel orange...")
                fluid_sim.start_pouring_liquid("miel", Color(1.0, 0.6, 0.1), 1420.0, 0.25, 30.0, 3.0)
            
            KEY_V:
                # Changer la fréquence de vérification de visibilité
                var current = fluid_sim.visibility_check_frequency
                var new_freq = 5 if current >= 10 else 10
                fluid_sim.visibility_check_frequency = new_freq
                print("\n[Touche V] Fréquence de visibilité: ", new_freq, " frames")
            
            KEY_R:
                # Vider le conteneur
                print("\n[Touche R] Vidage du conteneur...")
                fluid_sim.stop_pouring()
                fluid_sim.clear_particles()
                print("Conteneur vidé!")
            
            KEY_S:
                # Arrêter le versement
                if fluid_sim.is_pouring_active():
                    print("\n[Touche S] Arrêt du versement...")
                    fluid_sim.stop_pouring()

