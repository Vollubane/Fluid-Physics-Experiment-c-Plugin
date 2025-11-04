extends Node3D

@onready var fluid_sim = $FluidSimulator
@onready var multimesh_instance = $MultiMeshInstance3D

func _ready():
	# Les propriétés max_particles et visibility_check_frequency sont définies dans l'inspecteur
	print("=== Simulateur de fluides ===")
	print("Volume: ", fluid_sim.get_box_volume_liters(), " litres")
	print("Max particules: ", fluid_sim.max_particles)
	
	# Configurer le MultiMesh avec le bon rayon
	setup_multimesh()
	
	# Verser de l'eau bleue
	fluid_sim.start_pouring_liquid(
		"eau",                          # Nom du liquide
		Color(0.3, 0.6, 1.0, 1.0),     # Couleur (RGBA)
		1000.0,                         # Densité (kg/m³)
		0.0,                            # Restitution (aucun rebond)
		100.0,                          # Litres
		5.0                             # Durée (secondes)
	)

func setup_multimesh():
	# Obtenir le rayon réel des particules
	var particle_radius = fluid_sim.get_reference_particle_radius()
	print("Rayon des particules: ", particle_radius, " m")
	
	# Créer un nouveau MultiMesh
	var multimesh = MultiMesh.new()
	multimesh.transform_format = MultiMesh.TRANSFORM_3D
	multimesh.use_colors = true
	
	# Créer un mesh sphérique avec le bon rayon
	var sphere = SphereMesh.new()
	sphere.radius = particle_radius
	sphere.height = particle_radius * 2.0
	multimesh.mesh = sphere
	
	multimesh_instance.multimesh = multimesh
	print("MultiMesh configuré avec succès!")

func _process(delta):
	# Mettre à jour le MultiMesh avec les particules visibles
	update_multimesh()
	
	# Afficher les stats toutes les secondes
	if Engine.get_frames_drawn() % 60 == 0:
		var total = fluid_sim.get_particle_count()
		var visible = fluid_sim.get_visible_particle_count()
		var percent = (float(visible) / float(total) * 100.0) if total > 0 else 0.0
		print("Particules: %d total | %d visibles (%.1f%%)" % [total, visible, percent])

func update_multimesh():
	var total_count = fluid_sim.get_particle_count()
	
	if total_count == 0:
		multimesh_instance.multimesh.instance_count = 0
		return
	
	# Compter et placer seulement les particules visibles
	var visible_instances = []
	for i in range(total_count):
		if fluid_sim.is_particle_visible(i):
			visible_instances.append(i)
	
	multimesh_instance.multimesh.instance_count = visible_instances.size()
	
	# Mettre à jour les positions et couleurs
	for idx in range(visible_instances.size()):
		var particle_index = visible_instances[idx]
		var pos = fluid_sim.get_particle_position(particle_index)
		var color = fluid_sim.get_particle_color(particle_index)
		
		var transform = Transform3D(Basis(), pos)
		multimesh_instance.multimesh.set_instance_transform(idx, transform)
		multimesh_instance.multimesh.set_instance_color(idx, color)
