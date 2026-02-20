#include "voxel_shape.h"

#include "jolt_custom_shape_type.h"
#include "../spaces/jolt_query_collectors.h"

#include "Jolt/Physics/Collision/CastResult.h"
#include "Jolt/Physics/Collision/RayCast.h"
#include "Jolt/Physics/Collision/TransformedShape.h"
#include "Jolt/Physics/Collision/CollidePointResult.h"
#include "Jolt/Physics/Collision/CollisionCollectorImpl.h"
#include "Jolt/Physics/Collision/Shape/DecoratedShape.h"
#include "Jolt/Physics/Collision/CollisionDispatch.h"


// --- VoxelShapeSettings -- 
JPH::ShapeSettings::ShapeResult VoxelShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		new VoxelShape(*this, mCachedResult);

	return mCachedResult;
}

// --- VoxelShape ---
void VoxelShape::CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator &inSubShapeIDCreator,
		JPH::CollidePointCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter) const
{
	print_line("VOXEL CollidePoint");
	JPH_ASSERT(false);	// NOT IMPLEMENTED
}

int VoxelShape::GetIndex(uint32_t x, uint32_t y, uint32_t z) const
{
	// Z is slowest, Y is medium, X is fastest
	return (z * mResolution.GetY() * mResolution.GetX()) + (y * mResolution.GetX()) + x;
}

bool VoxelShape::IsSolidAt(const JPH::Vec3& voxelGridPos) const
{
	if (voxelGridPos.GetX() < 0 || voxelGridPos.GetX() >= mResolution.GetX() ||
			voxelGridPos.GetY() < 0 || voxelGridPos.GetY() >= mResolution.GetY() ||
			voxelGridPos.GetZ() < 0 || voxelGridPos.GetZ() >= mResolution.GetZ())
	{
		return false;
	}

	int64_t index = GetIndex((uint32_t)voxelGridPos.GetX(), (uint32_t)voxelGridPos.GetY(), (uint32_t)voxelGridPos.GetZ());

	uint8_t byteValue = mVoxelBitfieldData[index >> 3];
	uint8_t bitMask = (1 << (index & 7));

	return (byteValue & bitMask) != 0;
}

bool VoxelShape::CheckVoxelCollision(const JPH::Vec3 &voxelGridPos) const
{
	// Identify the 8 neighbors
	// We floor/ceil the local coordinates to find the surrounding voxel indices
	int minX = voxelGridPos.GetX();
	int minY = voxelGridPos.GetY();
	int minZ = voxelGridPos.GetZ();

	const uint8_t* voxelGrid = mVoxelBitfieldData;
	const size_t voxelGridSize = mVoxelBitfieldSize;

	for (int x = minX; x <= minX + 1; ++x) {
		for (int y = minY; y <= minY + 1; ++y) {
			for (int z = minZ; z <= minZ + 1; ++z) {
				const JPH::Vec3 pos = JPH::Vec3(x, y, z);

				// Check if the voxel at this point is solid.
				if (IsSolidAt(pos)) {
					return true;
				}
			}
		}
	}

	return false;
}

void VoxelShape::sCollideVoxelVsVoxelLocal(
		const VoxelShape *inShape1,
		const VoxelShape *inShape2,
		JPH::Mat44Arg inCenterOfMassTransform1,
		JPH::Mat44Arg inCenterOfMassTransform2,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator1,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
		const JPH::AABox &inIntersection,
		JPH::CollideShapeCollector &ioCollector)
{
	const uint8_t *datasets[] = { inShape1->mVoxelCornerData, inShape1->mVoxelEdgeData };
	const size_t sizes[] = { inShape1->mVoxelCornerDataSize, inShape1->mVoxelEdgeDataSize };

	// We want to move points from Shape 1's local space directly into Shape 2's local space.
	JPH::Mat44 transform1To2 = inCenterOfMassTransform2.Inversed() * inCenterOfMassTransform1;

	// Iterate over the corners first.
	// mVoxelCornerData is 4 bytes, x,y,z of the voxel grid position, the w is the encoded normal.
	const JPH::Vec4* corner_data = (const JPH::Vec4*)inShape1->mVoxelCornerData;
	int corner_data_size = (int)inShape1->mVoxelCornerDataSize / sizeof(JPH::Vec4);
	for (int i = 0; i < corner_data_size; i++)
	{
		// Transform the corner of Shape 1 into the local grid space of Shape 2
		const JPH::Vec3 pos(corner_data[i].GetX(), corner_data[i].GetY(), corner_data[i].GetZ());
		const JPH::Vec3 posIn2 = transform1To2 * pos;

		// Check if this corner point is "inside" any solid voxels in Shape 2
		if (inShape2->CheckVoxelCollision(posIn2))
		{
			// TODO:
#if 0
			JPH::CollideShapeResult result(
					worldPos1,
					worldPos2,
					-worldNormal, // Jolt expects normal pointing from 2 to 1
					penetrationDepth,
					inSubShapeIDCreator1.GetID(),
					inSubShapeIDCreator2.GetID(),
					JPH::TransformedShape::sGetBodyID(ioCollector.GetContext()));

			ioCollector.AddHit(result);
#endif
		}
	}
}

void VoxelShape::sCollideVoxelVsVoxel(const JPH::Shape *inShape1, const JPH::Shape *inShape2, JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
		JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
		const JPH::CollideShapeSettings &inCollideShapeSettings, JPH::CollideShapeCollector &ioCollector,
		const JPH::ShapeFilter &inShapeFilter)
{
	auto unwrap = [](const JPH::Shape *in) -> const VoxelShape * {
		const JPH::Shape *current = in;
		while (current->GetType() == JPH::EShapeType::Decorated) {
			current = static_cast<const JPH::DecoratedShape *>(current)->GetInnerShape();
		}
		return (current->GetSubType() == JoltCustomShapeSubType::VOXEL) ? static_cast<const VoxelShape *>(current) : nullptr;
	};

	const VoxelShape *shape1 = unwrap(inShape1);
	const VoxelShape *shape2 = unwrap(inShape2);
	if (!shape1 || !shape2) {
		return;
	}

	// AABB check
	JPH::AABox worldBounds1 = shape1->GetWorldSpaceBounds(inCenterOfMassTransform1, inScale1);
	JPH::AABox worldBounds2 = shape2->GetWorldSpaceBounds(inCenterOfMassTransform2, inScale2);
	JPH::AABox intersection = worldBounds1.Intersect(worldBounds2);
	if (!intersection.IsValid()) {
		return;
	}

	sCollideVoxelVsVoxelLocal(shape1, shape2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, intersection, ioCollector);
}

JPH::Vec3 VoxelShape::DecodeNormal(uint8_t mask) const
{
	static const JPH::Vec3 dirs[6] = {
		JPH::Vec3(0, 1, 0),		// UP
		JPH::Vec3(0, -1, 0),	// DOWN
		JPH::Vec3(-1, 0, 0),	// LEFT
		JPH::Vec3(1, 0, 0),		// RIGHT
		JPH::Vec3(0, 0, 1),		// FORWARD
		JPH::Vec3(0, 0, -1),	// BACK
	};

	JPH::Vec3 normal(0, 0, 0);
	for (int i = 0; i < 6; i++)
	{
		if (mask & (1 << i)) {
			normal += dirs[i];
		}
	}

	return normal.IsNearZero() ? JPH::Vec3(0, 1, 0) : normal.Normalized();
}

void VoxelShape::sRegister()
{
	JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL);
	f.mConstruct = []() -> Shape * { return new VoxelShape; };
	f.mColor = JPH::Color::sOrange;

	JPH::CollisionDispatch::sRegisterCollideShape(JoltCustomShapeSubType::VOXEL, JoltCustomShapeSubType::VOXEL, sCollideVoxelVsVoxel);
}
