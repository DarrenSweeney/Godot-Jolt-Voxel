#pragma once

#include "jolt_shape_3d.h"

class JoltVoxelShape3D final : public JoltShape3D
{
	Vector3 size; // Total size in meters
	Vector3i resolution; // Voxel grid dimensions
	PackedByteArray voxel_corner_data;
	PackedByteArray voxel_edge_data;
	PackedByteArray voxel_bitfield_data;
	float margin = 0.04f;

	virtual JPH::ShapeRefC _build() const override;

public:
	virtual ShapeType get_type() const override { return ShapeType::SHAPE_VOXEL; }
	virtual bool is_convex() const override { return false; }

	virtual Variant get_data() const override;
	virtual void set_data(const Variant &p_data) override;

	virtual float get_margin() const override { return margin; }
	virtual void set_margin(float p_margin) override;

	virtual AABB get_aabb() const override;

	String to_string() const;
};
