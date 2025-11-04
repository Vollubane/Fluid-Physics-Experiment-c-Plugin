#ifndef MARCHING_CUBES_HPP
#define MARCHING_CUBES_HPP

#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/vector3.hpp>

using namespace godot;

namespace MarchingCubes {
    // Tables de lookup pour Marching Cubes
    extern const int edge_table[256];
    extern const int tri_table[256][16];
    
    // Structure pour représenter une cellule de la grille
    struct GridCell {
        Vector3 position[8];  // Positions des 8 sommets de la cellule
        float value[8];       // Valeurs de densité aux 8 sommets
    };
    
    // Interpole entre deux points selon le seuil
    Vector3 vertex_interpolate(float isolevel, 
                               const Vector3 &p1, const Vector3 &p2,
                               float val1, float val2);
    
    // Génère les triangles pour une cellule donnée
    int polygonise(const GridCell &cell, float isolevel,
                   PackedVector3Array &vertices,
                   PackedInt32Array &indices);
}

#endif // MARCHING_CUBES_HPP

