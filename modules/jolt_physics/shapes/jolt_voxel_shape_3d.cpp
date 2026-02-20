#include "jolt_voxel_shape_3d.h"
#include "voxel_shape.h"

#include "../jolt_project_settings.h"
#include "../misc/jolt_type_conversions.h"

#include "Jolt/Physics/Collision/Shape/BoxShape.h"


JPH::ShapeRefC JoltVoxelShape3D::_build() const
{
	const float min_half_extent = (float)size[size.min_axis_index()];
	const float actual_margin = MIN(margin, min_half_extent * JoltProjectSettings::collision_margin_fraction);

#if 0
	const JPH::BoxShapeSettings shape_settings(to_jolt(size), actual_margin);
#else
	VoxelShapeSettings shape_settings;
	shape_settings.half_extents = to_jolt(size);
	shape_settings.resolution = JPH::Vec3((float)resolution.x, (float)resolution.y, (float)resolution.z);

	shape_settings.voxel_corner_data = voxel_corner_data.ptr();
	shape_settings.voxel_corner_data_size = (size_t)voxel_corner_data.size();

	shape_settings.voxel_edge_data = voxel_edge_data.ptr();
	shape_settings.voxel_edge_data_size = (size_t)voxel_edge_data.size();

	shape_settings.voxel_bitfield_data = voxel_bitfield_data.ptr();
	shape_settings.voxel_bitfield_data_size = (size_t)voxel_bitfield_data.size();
#endif


	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();
	ERR_FAIL_COND_V_MSG(shape_result.HasError(), nullptr, vformat("Failed to build Jolt Physics voxel shape with %s. It returned the following error: '%s'. This shape belongs to %s.", to_string(), to_godot(shape_result.GetError()), _owners_to_string()));

	return shape_result.Get();
}

void JoltVoxelShape3D::set_data(const Variant &p_data)
{
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);
	Dictionary d = p_data;

	const Vector3 new_half_extents = d["size"];
	const Vector3i new_resolution = d["resolution"];

	const PackedByteArray voxel_corners = d["voxel_corner_data"];
	const PackedByteArray voxel_edges = d["voxel_edge_data"];
	const PackedByteArray voxel_bitfield = d["voxel_bitfield_data"];

	// TODO: FIX THIS
	// TODO: Perf. Comparing two voxel datas could be very slow here
	//if (new_half_extents == size && new_resolution == resolution) //&& new_data == voxel_data) 
		//return;

	size = new_half_extents; // This is actually half_extents based on your Godot code
	resolution = new_resolution;

	voxel_corner_data = voxel_corners;
	voxel_edge_data = voxel_edges;
	voxel_bitfield_data = voxel_bitfield;

	destroy(); // Forces _build() to be called again
}

Variant JoltVoxelShape3D::get_data() const
{
	Dictionary d;
	d["size"] = size;
	d["resolution"] = resolution;
	d["voxel_corner_data"] = voxel_corner_data;
	d["voxel_edge_data"] = voxel_edge_data;
	d["voxel_bitfield_data"] = voxel_bitfield_data;
	return d;
}

void JoltVoxelShape3D::set_margin(float p_margin)
{
	if (unlikely(margin == p_margin)) {
		return;
	}

	margin = p_margin;

	destroy();
}

String JoltVoxelShape3D::to_string() const
{
	return vformat("{size=%v resolution=%v margin=%f}", size, resolution, margin);
}

AABB JoltVoxelShape3D::get_aabb() const
{
	return AABB(-size, size * 2.0f);
}
