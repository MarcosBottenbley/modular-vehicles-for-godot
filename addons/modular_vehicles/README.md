# Modular Vehicles for Godot

Native (GDExtension) raycast vehicle physics for Godot 4.4+ that supports any number of wheels.

This folder is the shippable addon. Extract a release zip at your project root so this folder
sits at `res://addons/modular_vehicles/`; no plugin needs to be enabled. Compiled libraries live
in `bin/` and come from GitHub Releases, or build them from source (see `docs/building.md` in
the repository).

Repository and documentation: https://github.com/MarcosBottenbley/modular-vehicles-for-godot

## Settings resources

Wheel tuning is split into shared `Resource` classes: `MVSurface` and `MVSurfaceLibrary` (grip per
node group), `MVTireSettings`, `MVSuspensionSettings` and `MVBrakeSettings`. Ready-made presets are
in `presets/`; every property, its default and unit is documented in `docs/settings.md` in the
repository.
