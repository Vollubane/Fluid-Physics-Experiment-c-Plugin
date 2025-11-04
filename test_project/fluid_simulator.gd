extends Node3D

@onready var fluid_sim = $FluidSimulator
@onready var multimesh_instance = $MultiMeshInstance3D

func _ready():
	setup_simulator()
	setup_multimesh()
	pour_initial_liquid()
	print_controls()

func setup_simulator():
	fluid_sim.max_particles = 10000
	fluid_sim.visibility_check_frequency = 10
	fluid_sim.gravity = Vector3(0, -9.81, 0)

func setup_multimesh():
	var multimesh = MultiMesh.new()
	multimesh.transform_format = MultiMesh.TRANSFORM_3D
	multimesh.use_colors = true
	
	var sphere = SphereMesh.new()
	sphere.radius = fluid_sim.get_reference_particle_radius()
	sphere.height = sphere.radius * 2
	multimesh.mesh = sphere
	
	# Matériau avec couleurs
	var material = StandardMaterial3D.new()
	material.vertex_color_use_as_albedo = true
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	sphere.material = material
	
	multimesh_instance.multimesh = multimesh

func pour_initial_liquid():
	fluid_sim.start_pouring_liquid("eau", Color(0.3, 0.6, 1.0), 1000.0, 0.4, 100.0, 3.0)

func print_controls():
	print("\n=== CONTRÔLES ===")
	print("1 - Verser de l'eau")
	print("2 - Verser de l'huile")
	print("3 - Verser du colorant rouge")
	print("4 - Verser du colorant vert")
	print("R - Vider le conteneur")
	print("S - Arrêter le versement")
	print("================\n")

func _process(delta):
	update_multimesh()
	
	# Stats toutes les secondes
	if Engine.get_frames_drawn() % 60 == 0:
		print_stats()

func update_multimesh():
	var total = fluid_sim.get_particle_count()
	if total == 0:
		multimesh_instance.multimesh.instance_count = 0
		return
	
	# Collecter les indices des particules visibles
	var visible_indices = []
	for i in range(total):
		if fluid_sim.is_particle_visible(i):
			visible_indices.append(i)
	
	multimesh_instance.multimesh.instance_count = visible_indices.size()
	
	# Mettre à jour les transforms et couleurs
	for idx in range(visible_indices.size()):
		var i = visible_indices[idx]
		var pos = fluid_sim.get_particle_position(i)
		var color = fluid_sim.get_particle_color(i)
		
		multimesh_instance.multimesh.set_instance_transform(idx, Transform3D(Basis(), pos))
		multimesh_instance.multimesh.set_instance_color(idx, color)

func print_stats():
	var total = fluid_sim.get_particle_count()
	var visible = fluid_sim.get_visible_particle_count()
	var percent = (float(visible) / float(total) * 100.0) if total > 0 else 0.0
	
	print("Particules: %d total | %d visibles (%.1f%%)" % [total, visible, percent])

func _input(event):
	if event is InputEventKey and event.pressed:
		match event.keycode:
			KEY_1:
				fluid_sim.start_pouring_liquid("eau", Color(0.3, 0.6, 1.0), 1000.0, 0.4, 50.0, 2.0)
				print("→ Versement d'eau")
			
			KEY_2:
				fluid_sim.start_pouring_liquid("huile", Color(1.0, 0.9, 0.2), 920.0, 0.3, 50.0, 2.0)
				print("→ Versement d'huile")
			
			KEY_3:
				fluid_sim.start_pouring_liquid("colorant_rouge", Color(1.0, 0.2, 0.2), 1000.0, 0.4, 10.0, 1.0)
				print("→ Versement de colorant rouge")
			
			KEY_4:
				fluid_sim.start_pouring_liquid("colorant_vert", Color(0.2, 1.0, 0.3), 1000.0, 0.4, 10.0, 1.0)
				print("→ Versement de colorant vert")
			
			KEY_R:
				fluid_sim.clear_particles()
				print("→ Conteneur vidé")
			
			KEY_S:
				fluid_sim.stop_pouring()
				print("→ Versement arrêté")
