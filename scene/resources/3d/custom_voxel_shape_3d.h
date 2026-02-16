#pragma once

#include "scene/resources/3d/shape_3d.h"

class CustomVoxelShape3D : public Shape3D {
	GDCLASS(CustomVoxelShape3D, Shape3D);

private:
	Vector3 size = Vector3(1, 1, 1);
	Vector3i resolution;
	float voxel_size_per_meter;
	PackedByteArray voxel_data; // classification data

	_FORCE_INLINE_ int _get_index(int p_x, int p_y, int p_z) const {
		// Matches: y * (width * depth) + z * width + x
		return p_y * (resolution.x * resolution.z) + p_z * resolution.x + p_x;
	}

protected:
	static void _bind_methods();

	virtual void _update_shape() override;

public:
	void set_size(const Vector3 &p_size);
	Vector3 get_size() const;

	// Methods to interact with the voxels
	void set_voxel(const Vector3i &p_pos, uint8_t p_value);
	uint8_t get_voxel(const Vector3i &p_pos) const;

	void set_voxel_data(const PackedByteArray &p_data);
	PackedByteArray get_voxel_data() const;

	void set_resolution(const Vector3i &p_res);
	Vector3i get_resolution() const { return resolution; }

	virtual Vector<Vector3> get_debug_mesh_lines() const override;
	virtual Ref<ArrayMesh> get_debug_arraymesh_faces(const Color &p_modulate) const override;
	virtual real_t get_enclosing_radius() const override;

	CustomVoxelShape3D();
};
