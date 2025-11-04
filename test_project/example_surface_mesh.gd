extends Node3D

@onready var fluid_sim = $FluidSimulator
@onready var mesh_instance = $MeshInstance3D

var update_mesh_timer = 0.0
var mesh_update_interval = 0.5  # Mettre à jour le mesh toutes les 0.5 secondes

func _ready():
	print("=== Test de reconstruction de surface ===")
	
	# Configurer le simulateur
	fluid_sim.max_particles = 2000
	fluid_sim.surface_mesh_enabled = true
	fluid_sim.surface_threshold = 0.6  # Ajustez ce paramètre pour l'apparence
	fluid_sim.surface_grid_resolution = 0.03  # 3cm de résolution de grille
	
	print("Paramètres:")
	print("  - Max particules: ", fluid_sim.max_particles)
	print("  - Seuil de surface: ", fluid_sim.surface_threshold)
	print("  - Résolution de grille: ", fluid_sim.surface_grid_resolution, " m")
	
	# Verser du liquide
	fluid_sim.start_pouring_liquid(
		"eau",                          # Nom du liquide
		Color(0.3, 0.6, 1.0, 0.8),     # Couleur (RGBA) - semi-transparent
		1000.0,                         # Densité (kg/m³)
		0.0,                            # Restitution (aucun rebond)
		50.0,                           # Litres
		3.0                             # Durée (secondes)
	)
	
	# Créer un matériau pour le mesh
	var material = StandardMaterial3D.new()
	material.albedo_color = Color(0.3, 0.6, 1.0, 0.8)
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.cull_mode = BaseMaterial3D.CULL_DISABLED  # Visible des deux côtés
	material.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	mesh_instance.material_override = material

func _process(delta):
	update_mesh_timer += delta
	
	# Mettre à jour le mesh périodiquement (pas chaque frame car c'est coûteux)
	if update_mesh_timer >= mesh_update_interval:
		update_mesh_timer = 0.0
		update_surface_mesh()
	
	# Stats toutes les secondes
	if Engine.get_frames_drawn() % 60 == 0:
		var particle_count = fluid_sim.get_particle_count()
		print("Particules: ", particle_count)

func update_surface_mesh():
	# Générer le mesh de surface
	var mesh_data = fluid_sim.generate_surface_mesh()
	
	if mesh_data.size() != 2:
		return
	
	var vertices = mesh_data[0]  # PackedVector3Array
	var indices = mesh_data[1]   # PackedInt32Array
	
	if vertices.size() == 0:
		# Pas de surface à afficher
		mesh_instance.mesh = null
		return
	
	# Créer un ArrayMesh
	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_INDEX] = indices
	
	# Calculer les normales automatiquement
	var normals = PackedVector3Array()
	normals.resize(vertices.size())
	
	# Calculer les normales par face
	for i in range(0, indices.size(), 3):
		var idx0 = indices[i]
		var idx1 = indices[i + 1]
		var idx2 = indices[i + 2]
		
		var v0 = vertices[idx0]
		var v1 = vertices[idx1]
		var v2 = vertices[idx2]
		
		var edge1 = v1 - v0
		var edge2 = v2 - v0
		var normal = edge1.cross(edge2).normalized()
		
		normals[idx0] += normal
		normals[idx1] += normal
		normals[idx2] += normal
	
	# Normaliser les normales
	for i in range(normals.size()):
		normals[i] = normals[i].normalized()
	
	arrays[Mesh.ARRAY_NORMAL] = normals
	
	# Créer le mesh
	var array_mesh = ArrayMesh.new()
	array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	
	mesh_instance.mesh = array_mesh
	
	print("Mesh mis à jour: ", vertices.size(), " vertices, ", indices.size() / 3, " triangles")

func _input(event):
	if event.is_action_pressed("ui_accept"):  # Espace
		print("=== Versement d'eau supplémentaire ===")
		fluid_sim.start_pouring_liquid(
			"eau", Color(0.3, 0.6, 1.0, 0.8), 1000.0, 0.0, 20.0, 2.0
		)
	
	if event.is_action_pressed("ui_cancel"):  # Escape
		print("=== Nettoyage ===")
		fluid_sim.clear_particles()

