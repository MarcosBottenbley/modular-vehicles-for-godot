#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "mv_version.h"
#include "nodes/mv_vehicle.h"

using namespace godot;

void initialize_modular_vehicles_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	// Resources are registered before the nodes that reference them.
	GDREGISTER_CLASS(MVVehicle);

	UtilityFunctions::print("Modular Vehicles ", MV_VERSION, " (", MV_BUILD_TARGET, ") loaded");
}

void uninitialize_modular_vehicles_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}

extern "C" {
// Entry point named in addons/modular_vehicles/modular_vehicles.gdextension.
GDExtensionBool GDE_EXPORT modular_vehicles_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_modular_vehicles_module);
	init_obj.register_terminator(uninitialize_modular_vehicles_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

	return init_obj.init();
}
}
