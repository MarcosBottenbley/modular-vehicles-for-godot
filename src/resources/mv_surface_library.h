#pragma once

// Terms:
//   surface: an MVSurface, the grip numbers for one kind of ground (see mv_surface.h).
//   surface library: the ordered list of surfaces one vehicle knows about plus its fallback.
//   lookup name: the node group name a wheel read from the collider it touches.
//   default surface: the surface returned when no entry matches the lookup name.
//   built-in road: one MVSurface with default values shared by every library whose default
//     surface is empty, so a lookup never returns null. Lives until the extension unloads.

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/variant/typed_array.hpp>

#include "mv_surface.h"

namespace godot {

// The surface library of a vehicle. MVVehicle hands it to its MVWheel descendants, and each
// wheel calls find_surface() with the lookup name whenever the body under it changes.
class MVSurfaceLibrary : public Resource {
	GDCLASS(MVSurfaceLibrary, Resource)

	TypedArray<MVSurface> surfaces;
	Ref<MVSurface> default_surface;

	static Ref<MVSurface> builtin_road;

protected:
	static void _bind_methods();

public:
	// Replaces the list of surfaces.
	// p_surfaces: surfaces in lookup order; null entries are skipped by find_surface().
	// Returns: nothing. Emits changed; wheels redo their lookup on the next physics tick.
	void set_surfaces(const TypedArray<MVSurface> &p_surfaces);
	// Returns: the surfaces in lookup order.
	TypedArray<MVSurface> get_surfaces() const;

	// Sets the default surface.
	// p_default_surface: fallback for unknown lookup names; may be null to use the built-in road.
	// Returns: nothing. Emits changed; wheels redo their lookup on the next physics tick.
	void set_default_surface(const Ref<MVSurface> &p_default_surface);
	// Returns: the default surface, or null when the built-in road is the fallback.
	Ref<MVSurface> get_default_surface() const;

	// Resolves a lookup name to a surface.
	// p_name: lookup name, compared exactly against each surface's name.
	// Returns: the first surface with that name, else the default surface, else the built-in
	// road. Never null; MVWheel keeps the returned Ref and reads it every physics tick.
	Ref<MVSurface> find_surface(const StringName &p_name) const;

	// Returns: the built-in road surface, created on first use.
	static Ref<MVSurface> get_builtin_road();

	// Drops the built-in road. The library is reloadable, so the static Ref must be released
	// from uninitialize_modular_vehicles_module() before Godot unloads the DLL.
	// Returns: nothing. The next get_builtin_road() creates a fresh instance.
	static void release_builtin_surfaces();
};

} // namespace godot
