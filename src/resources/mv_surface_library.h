#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/variant/typed_array.hpp>

#include "mv_surface.h"

namespace godot {

// The set of surfaces a vehicle knows about. find_surface() maps a node group
// name to a surface, falling back to default_surface and finally to a shared
// built-in "Road" surface so a wheel always has valid data.
class MVSurfaceLibrary : public Resource {
	GDCLASS(MVSurfaceLibrary, Resource)

	TypedArray<MVSurface> surfaces;
	Ref<MVSurface> default_surface;

	static Ref<MVSurface> builtin_road;

protected:
	static void _bind_methods();

public:
	void set_surfaces(const TypedArray<MVSurface> &p_surfaces);
	TypedArray<MVSurface> get_surfaces() const;

	void set_default_surface(const Ref<MVSurface> &p_default_surface);
	Ref<MVSurface> get_default_surface() const;

	Ref<MVSurface> find_surface(const StringName &p_name) const;

	static Ref<MVSurface> get_builtin_road();
	// Called from the extension terminator; the static Ref must not outlive Godot.
	static void release_builtin_surfaces();
};

} // namespace godot
