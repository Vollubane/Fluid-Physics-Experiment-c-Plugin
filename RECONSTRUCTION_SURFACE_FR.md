# Reconstruction de Surface (Marching Cubes)

Le système de reconstruction de surface permet de générer un mesh 3D fluide et lisse à partir des particules, créant un rendu de liquide réaliste.

## Algorithme : Marching Cubes + Metaballs

### Principe

1. **Metaballs** : Chaque particule génère un champ de densité autour d'elle
2. **Grille 3D** : L'espace est divisé en cellules cubiques
3. **Marching Cubes** : Pour chaque cellule, on calcule la surface où la densité = seuil
4. **Mesh final** : Tous les triangles sont combinés en un seul mesh

### Avantages

✅ Surface lisse et fluide  
✅ Pas besoin de shaders complexes  
✅ Supporte les normales pour éclairage réaliste  
✅ Utilise le spatial hash existant (optimisé)  

### Inconvénients

⚠️ Calcul CPU coûteux (ne pas mettre à jour chaque frame)  
⚠️ Mieux adapté pour < 5000 particules  

---

## Utilisation dans Godot

### 1. Activer la reconstruction de surface

Dans l'inspecteur du `FluidSimulator` :

```
surface_mesh_enabled = true
surface_threshold = 0.6          # Plus élevé = surface plus proche des particules
surface_grid_resolution = 0.03   # Plus petit = plus de détails (mais plus lent)
```

### 2. Générer le mesh

En GDScript :

```gdscript
var mesh_data = fluid_sim.generate_surface_mesh()

if mesh_data.size() == 2:
    var vertices = mesh_data[0]  # PackedVector3Array
    var indices = mesh_data[1]   # PackedInt32Array
    
    # Créer un ArrayMesh
    var arrays = []
    arrays.resize(Mesh.ARRAY_MAX)
    arrays[Mesh.ARRAY_VERTEX] = vertices
    arrays[Mesh.ARRAY_INDEX] = indices
    
    var array_mesh = ArrayMesh.new()
    array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
    
    mesh_instance.mesh = array_mesh
```

### 3. Exemple complet

Voir `test_project/example_surface_mesh.gd` et `test_project/surface_mesh_test.tscn`

**Commandes** :
- `Espace` : Verser plus de liquide
- `Escape` : Effacer les particules

---

## Paramètres à ajuster

### `surface_threshold` (0.0 - 2.0)

- **Plus bas (0.3)** : Surface gonflée, englobe plus de volume
- **Moyen (0.6)** : Équilibre (recommandé)
- **Plus haut (1.2)** : Surface serrée, proche des particules

### `surface_grid_resolution` (0.005 - 0.1 m)

- **Petite (0.01)** : Beaucoup de détails, lent
- **Moyenne (0.03)** : Bon compromis
- **Grande (0.05)** : Rapide mais moins de détails

### Fréquence de mise à jour

**Ne pas appeler `generate_surface_mesh()` chaque frame !**

Recommandation :
- 30 FPS = toutes les 2 frames (0.066s)
- 60 FPS = toutes les 4-5 frames (0.1s)
- Pour > 2000 particules : toutes les 0.5s

---

## Combiner avec les particules

Vous pouvez afficher **à la fois** :

1. **Le mesh de surface** (reconstruction lisse)
2. **Les particules visibles** (MultiMesh pour les éclaboussures)

Exemple :

```gdscript
func _process(delta):
    # Mesh de surface (lent)
    if timer > 0.1:
        update_surface_mesh()
        timer = 0.0
    
    # Particules visibles (rapide)
    update_particle_multimesh()
```

---

## Optimisation

### Pour grandes simulations (> 5000 particules)

1. **Augmenter `surface_grid_resolution`** à 0.05 ou 0.1
2. **Réduire la fréquence** de mise à jour (0.5s - 1.0s)
3. **Limiter la zone de reconstruction** (TODO : ajouter paramètre `bounds`)

### Alternative : Screen Space Fluids

Pour > 10 000 particules, envisagez d'implémenter un shader de type **Screen Space Fluids** (rendu GPU) si les performances CPU sont insuffisantes.

---

## Notes techniques

### Fonction de densité metaball

Utilisée : **Cubic falloff** (lisse et rapide)

```cpp
f(r) = (1 - (r/R)³)³  si r < R, sinon 0
```

Où `R = particle_radius * 3.0` (rayon d'influence)

### Tables de Marching Cubes

256 configurations possibles (2^8 sommets)  
Implémentation dans `src/marching_cubes.cpp`

---

## Prochaines améliorations possibles

- [ ] Calcul des normales lissées (smooth shading)
- [ ] Support des UV pour textures
- [ ] Couleur par vertex (mélange de liquides)
- [ ] Reconstruction par zone (optimisation)
- [ ] Threading pour génération parallèle

---

## Exemple d'utilisation

```gdscript
extends Node3D

@onready var fluid_sim = $FluidSimulator
@onready var mesh_instance = $MeshInstance3D

func _ready():
    # Configuration
    fluid_sim.max_particles = 2000
    fluid_sim.surface_mesh_enabled = true
    fluid_sim.surface_threshold = 0.6
    fluid_sim.surface_grid_resolution = 0.03
    
    # Verser du liquide
    fluid_sim.start_pouring_liquid("eau", Color.BLUE, 1000.0, 0.0, 50.0, 3.0)
    
    # Matériau
    var mat = StandardMaterial3D.new()
    mat.albedo_color = Color(0.3, 0.6, 1.0, 0.8)
    mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    mesh_instance.material_override = mat

var timer = 0.0

func _process(delta):
    timer += delta
    if timer > 0.1:  # 10 FPS de mise à jour du mesh
        timer = 0.0
        var data = fluid_sim.generate_surface_mesh()
        if data.size() == 2:
            create_mesh(data[0], data[1])

func create_mesh(vertices, indices):
    var arrays = []
    arrays.resize(Mesh.ARRAY_MAX)
    arrays[Mesh.ARRAY_VERTEX] = vertices
    arrays[Mesh.ARRAY_INDEX] = indices
    
    var mesh = ArrayMesh.new()
    mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
    mesh_instance.mesh = mesh
```

---

**Bon rendu de fluide ! 🌊**

