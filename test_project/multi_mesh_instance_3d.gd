extends MultiMeshInstance3D

@onready var fluid_sim = get_node("../FluidSimulator")

func _ready():
	# Configurer le simulateur AVANT de créer le mesh
	fluid_sim.max_particles = 20000
	
	# Créer le mesh avec le bon rayon
	setup_multimesh()
	
	# Verser du liquide
	fluid_sim.start_pouring_liquid("eau", Color(0.3, 0.6, 1.0), 1000.0, 0.4, 50.0, 3.0)

func setup_multimesh():
	# Obtenir le rayon réel des particules (recalculé automatiquement)
	var particle_radius = fluid_sim.get_reference_particle_radius()
	print("Configuration du mesh avec rayon: ", particle_radius, " m (max_particles: ", fluid_sim.max_particles, ")")
	
	# Créer un nouveau MultiMesh
	var multimesh = MultiMesh.new()
	multimesh.transform_format = MultiMesh.TRANSFORM_3D
	multimesh.use_colors = true  # Important pour les couleurs
	
	# Créer un mesh sphérique avec le bon rayon
	var sphere = SphereMesh.new()
	sphere.radius = particle_radius
	sphere.height = particle_radius * 2.0
	multimesh.mesh = sphere
	
	self.multimesh = multimesh
	print("Mesh créé avec succès!")

func _process(delta):
	# Mettre à jour seulement les particules visibles
	var visible_count = fluid_sim.get_visible_particle_count()
	var total_count = fluid_sim.get_particle_count()
	
	if total_count == 0:
		self.multimesh.instance_count = 0
		return
	
	# Compter et placer seulement les particules visibles
	var visible_instances = []
	for i in range(total_count):
		if fluid_sim.is_particle_visible(i):
			visible_instances.append(i)
	
	self.multimesh.instance_count = visible_instances.size()
	
	# Mettre à jour les positions et couleurs
	for idx in range(visible_instances.size()):
		var particle_index = visible_instances[idx]
		var pos = fluid_sim.get_particle_position(particle_index)
		var color = fluid_sim.get_particle_color(particle_index)
		
		var transform = Transform3D(Basis(), pos)
		self.multimesh.set_instance_transform(idx, transform)
		self.multimesh.set_instance_color(idx, color)
