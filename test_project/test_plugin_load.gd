extends Node

func _ready():
	print("=== TEST DE CHARGEMENT DU PLUGIN ===")
	
	# Vérifie si la classe FluidSimulator existe
	if ClassDB.class_exists("FluidSimulator"):
		print("✓ FluidSimulator est chargé!")
		
		# Essaie de créer une instance
		var fs = ClassDB.instantiate("FluidSimulator")
		if fs:
			print("✓ Instance créée avec succès!")
			print("Type: ", fs.get_class())
			fs.queue_free()
		else:
			print("✗ Impossible de créer une instance")
	else:
		print("✗ FluidSimulator n'est PAS chargé!")
		print("Classes disponibles contenant 'Fluid':")
		for cls in ClassDB.get_class_list():
			if "Fluid" in cls or "fluid" in cls:
				print("  - ", cls)
	
	# Vérifie si ItemData existe (pour comparer)
	if ClassDB.class_exists("ItemData"):
		print("✓ ItemData est chargé (pour comparaison)")
	else:
		print("✗ ItemData n'est PAS chargé non plus!")
	
	print("=== FIN DU TEST ===")
