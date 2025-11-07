extends Control

@onready var fluid_sim = get_node("/root/Main/Node3D/FluidSimulator")

@onready var liquid_name_input = $Panel/VBoxContainer/LiquidName/LineEdit
@onready var color_picker = $Panel/VBoxContainer/ColorPicker
@onready var density_input = $Panel/VBoxContainer/Density/SpinBox
@onready var restitution_input = $Panel/VBoxContainer/Restitution/SpinBox
@onready var viscosity_slider = $Panel/VBoxContainer/Viscosity/HSlider
@onready var viscosity_label = $Panel/VBoxContainer/Viscosity/Label
@onready var pour_button = $Panel/VBoxContainer/PourButton

func _ready():
	# Valeurs par défaut (eau)
	liquid_name_input.text = "eau"
	color_picker.color = Color(0.3, 0.6, 1.0, 1.0)
	density_input.value = 1000.0
	restitution_input.value = 0.0
	viscosity_slider.value = fluid_sim.viscosity
	_update_viscosity_label()
	
	pour_button.pressed.connect(_on_pour_button_pressed)
	viscosity_slider.value_changed.connect(_on_viscosity_changed)

func _update_viscosity_label():
	viscosity_label.text = "Viscosité: %.1f" % viscosity_slider.value

func _on_viscosity_changed(value):
	fluid_sim.viscosity = value
	_update_viscosity_label()

func _on_pour_button_pressed():
	var liquid_name = liquid_name_input.text
	var color = color_picker.color
	var density = density_input.value
	var restitution = restitution_input.value
	
	# Verser 50L en 3 secondes
	fluid_sim.start_pouring_liquid(liquid_name, color, density, restitution, 50.0, 3.0)
	
	print("Versement de 50L de %s (densité: %.1f kg/m³, restitution: %.2f)" % [liquid_name, density, restitution])
