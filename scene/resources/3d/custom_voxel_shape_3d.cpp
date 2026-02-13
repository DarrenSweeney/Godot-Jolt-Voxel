#include "custom_voxel_shape_3d.h"

#include "scene/resources/3d/primitive_meshes.h" // TODO: For now
#include "servers/physics_3d/physics_server_3d.h"

CustomVoxelShape3D::CustomVoxelShape3D() :
		Shape3D(PhysicsServer3D::get_singleton()->shape_create(PhysicsServer3D::SHAPE_VOXEL)) {
}


void CustomVoxelShape3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_size", "size"), &CustomVoxelShape3D::set_size);
	ClassDB::bind_method(D_METHOD("get_size"), &CustomVoxelShape3D::get_size);

	ClassDB::bind_method(D_METHOD("set_resolution", "resolution"), &CustomVoxelShape3D::set_resolution);
	ClassDB::bind_method(D_METHOD("get_resolution"), &CustomVoxelShape3D::get_resolution);

	ClassDB::bind_method(D_METHOD("set_voxel_data", "data"), &CustomVoxelShape3D::set_voxel_data);
	ClassDB::bind_method(D_METHOD("get_voxel_data"), &CustomVoxelShape3D::get_voxel_data);

	// Bind Properties (This makes them show up in the Inspector and allows dot syntax)
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size"), "set_size", "get_size");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3I, "resolution"), "set_resolution", "get_resolution");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_BYTE_ARRAY, "voxel_data"), "set_voxel_data", "get_voxel_data");
}

void CustomVoxelShape3D::_update_shape() {
	if (voxel_data.size() != (resolution.x * resolution.y * resolution.z)) {
		// TODO: Maybe assert here?
	}

	Dictionary d;
	d["size"] = size / 2;
	d["resolution"] = resolution;
	d["data"] = voxel_data;

	PhysicsServer3D::get_singleton()->shape_set_data(get_shape(), d);
	Shape3D::_update_shape();
}

void CustomVoxelShape3D::set_size(const Vector3 &p_size) {
	size = p_size;
	emit_changed();
}

Vector3 CustomVoxelShape3D::get_size() const {
	return size;
}

void CustomVoxelShape3D::set_voxel_data(const PackedByteArray &p_data) {
	voxel_data = p_data;
	_update_shape();
	emit_changed();
}

PackedByteArray CustomVoxelShape3D::get_voxel_data() const {
	return voxel_data;
}

void CustomVoxelShape3D::set_resolution(const Vector3i &p_res) {
	ERR_FAIL_COND(p_res.x <= 0 || p_res.y <= 0 || p_res.z <= 0);
	resolution = p_res;
	voxel_data.resize(resolution.x * resolution.y * resolution.z);
	_update_shape();
	emit_changed();
}

void CustomVoxelShape3D::set_voxel(const Vector3i &p_pos, uint8_t p_value) {
	ERR_FAIL_INDEX(p_pos.x, resolution.x);
	ERR_FAIL_INDEX(p_pos.y, resolution.y);
	ERR_FAIL_INDEX(p_pos.z, resolution.z);

	int idx = _get_index(p_pos.x, p_pos.y, p_pos.z);
	voxel_data.write[idx] = p_value;

	// In voxel physics, changing one pixel is a shape change!
	_update_shape();
	emit_changed();
}

uint8_t CustomVoxelShape3D::get_voxel(const Vector3i &p_pos) const {
	int idx = _get_index(p_pos.x, p_pos.y, p_pos.z);
	return voxel_data[idx];
}

Vector<Vector3> CustomVoxelShape3D::get_debug_mesh_lines() const {
	Vector<Vector3> lines;
	AABB aabb;
	aabb.position = -size / 2;
	aabb.size = size;

	for (int i = 0; i < 12; i++) {
		Vector3 a, b;
		aabb.get_edge(i, a, b);
		lines.push_back(a);
		lines.push_back(b);
	}

	return lines;
}

// NOTE: Just for debugging
//Vector<Vector3> CustomVoxelShape3D::get_debug_mesh_lines() const {
//	// Keep the main bounding box lines
//	Vector<Vector3> lines;
//
//	// Draw tiny markers for solid voxels (Verification only!)
//	if (voxel_data.size() > 0) {
//		Vector3 cell_dim = size / Vector3(resolution);
//		Vector3 offset = -size / 2.0 + (cell_dim / 2.0);
//
//		for (int z = 0; z < resolution.z; z++) {
//			for (int y = 0; y < resolution.y; y++) {
//				for (int x = 0; x < resolution.x; x++) {
//					if (get_voxel(Vector3i(x, y, z)) > 0) {
//						Vector3 center = offset + Vector3(x, y, z) * cell_dim;
//						// Draw a small 0.1m cross at the center of each solid voxel
//						lines.push_back(center + Vector3(0.1, 0, 0));
//						lines.push_back(center - Vector3(0.1, 0, 0));
//						lines.push_back(center + Vector3(0, 0.1, 0));
//						lines.push_back(center - Vector3(0, 0.1, 0));
//					}
//				}
//			}
//		}
//	}
//	return lines;
//}

Ref<ArrayMesh> CustomVoxelShape3D::get_debug_arraymesh_faces(const Color &p_modulate) const {
	Array box_array;
	box_array.resize(RS::ARRAY_MAX);
	BoxMesh::create_mesh_array(box_array, size);

	Vector<Color> colors;
	const PackedVector3Array &verts = box_array[RS::ARRAY_VERTEX];
	const int32_t verts_size = verts.size();
	for (int i = 0; i < verts_size; i++) {
		colors.append(p_modulate);
	}

	Ref<ArrayMesh> box_mesh = memnew(ArrayMesh);
	box_array[RS::ARRAY_COLOR] = colors;
	box_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, box_array);
	return box_mesh;
}

real_t CustomVoxelShape3D::get_enclosing_radius() const {
	return size.length() / 2;
}
