// Terms: see mv_surface_library.h.

#include "mv_surface_library.h"

#include <godot_cpp/core/class_db.hpp>

#include "mv_settings_helpers.h"

using namespace godot;

Ref<MVSurface> MVSurfaceLibrary::builtin_road;

void MVSurfaceLibrary::set_surfaces(const TypedArray<MVSurface> &p_surfaces) {
	// Arrays are shared references, so an equality test cannot tell an edit from a no-op;
	// always emit.
	surfaces = p_surfaces;
	emit_changed();
}

TypedArray<MVSurface> MVSurfaceLibrary::get_surfaces() const {
	return surfaces;
}

void MVSurfaceLibrary::set_default_surface(const Ref<MVSurface> &p_default_surface) {
	if (mv_store_if_changed(default_surface, p_default_surface)) {
		emit_changed();
	}
}

Ref<MVSurface> MVSurfaceLibrary::get_default_surface() const {
	return default_surface;
}

Ref<MVSurface> MVSurfaceLibrary::find_surface(const StringName &p_name) const {
	for (int surface_index = 0; surface_index < surfaces.size(); surface_index++) {
		Ref<MVSurface> surface = surfaces[surface_index];
		if (surface.is_valid() && surface->get_name() == p_name) {
			return surface;
		}
	}
	if (default_surface.is_valid()) {
		return default_surface;
	}
	return get_builtin_road();
}

Ref<MVSurface> MVSurfaceLibrary::get_builtin_road() {
	if (builtin_road.is_null()) {
		builtin_road.instantiate();
	}
	return builtin_road;
}

void MVSurfaceLibrary::release_builtin_surfaces() {
	builtin_road.unref();
}

void MVSurfaceLibrary::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_surfaces", "surfaces"), &MVSurfaceLibrary::set_surfaces);
	ClassDB::bind_method(D_METHOD("get_surfaces"), &MVSurfaceLibrary::get_surfaces);
	ClassDB::bind_method(D_METHOD("set_default_surface", "default_surface"), &MVSurfaceLibrary::set_default_surface);
	ClassDB::bind_method(D_METHOD("get_default_surface"), &MVSurfaceLibrary::get_default_surface);
	ClassDB::bind_method(D_METHOD("find_surface", "name"), &MVSurfaceLibrary::find_surface);

	// "24/17:MVSurface" tells the inspector the array holds Object resources of type MVSurface.
	const String surface_array_hint = vformat("%s/%s:MVSurface", Variant::OBJECT, PROPERTY_HINT_RESOURCE_TYPE);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "surfaces", PROPERTY_HINT_ARRAY_TYPE, surface_array_hint), "set_surfaces", "get_surfaces");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "default_surface", PROPERTY_HINT_RESOURCE_TYPE, "MVSurface"), "set_default_surface", "get_default_surface");
}
