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

bool VoxelShape::IsSolidAt(int inX, int inY, int inZ) const
{
	if (inX < 0 || inX >= mResolution.GetX() ||
			inY < 0 || inY >= mResolution.GetY() ||
			inZ < 0 || inZ >= mResolution.GetZ()) {
		return false;
	}

	int64_t index = GetIndex((uint32_t)inX, (uint32_t)inY, (uint32_t)inZ);

	uint8_t byteValue = mVoxelBitfieldData[index >> 3];
	uint8_t bitMask = (1 << (index & 7));

	return (byteValue & bitMask) != 0;
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
	const uint8_t *corner_data = inShape1->mVoxelCornerData;
	int corner_data_size = (int)inShape1->mVoxelCornerDataSize;
	for (int i = 0; i < corner_data_size; i++)
	{

	}

#if 0
	for (int d = 0; d < 2; d++) {
		const uint8_t *data = datasets[d];
		size_t size = sizes[d];
		if (!data || size == 0) {
			continue;
		}

		for (size_t i = 0; i + 3 < size; i += 4) {
			// 1. Get Point from Shape 1 in World Space
			JPH::Vec3 localPos1 = inShape1->GetLocalPos(data[i], data[i + 1], data[i + 2]);
			JPH::Vec3 worldPos = inCenterOfMassTransform1 * localPos1;

			// 2. Early out if not in intersection AABB
			if (!inIntersection.Contains(worldPos)) {
				continue;
			}

			// 3. Transform World Point to Shape 2's Local Space
			JPH::Vec3 localPos2 = invTransform2 * worldPos;

			// 4. THE VOXEL CHECK: Is there a voxel in Shape 2 at this position?
			// Convert local float position to integer grid coordinates
			JPH::Vec3 coord = inShape2->GetVoxelCoord(localPos2);

			if (inShape2->IsSolidAt((int)coord.GetX(), (int)coord.GetY(), (int)coord.GetZ()))
			{
				// 5. Build the Result
				JPH::Vec3 localNormal = inShape1->DecodeNormal(data[i + 3]);
				JPH::Vec3 worldNormal = inCenterOfMassTransform1.Multiply3x3(localNormal);

				JPH::CollideShapeResult result;
				result.mContactPointOn1 = worldPos;
				result.mContactPointOn2 = worldPos;

				result.mSubShapeID1 = inSubShapeIDCreator2.GetID();
				result.mSubShapeID2 = inSubShapeIDCreator1.GetID();
				result.mPenetrationAxis = worldNormal
				result.mPenetrationDepth = 0.1f;

				ioCollector.AddHit(result);

				if (ioCollector.ShouldEarlyOut()) {
					return;
				}
			}
		}
	}
#endif
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

void VoxelShape::sRegister()
{
	JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL);
	f.mConstruct = []() -> Shape * { return new VoxelShape; };
	f.mColor = JPH::Color::sOrange;

	JPH::CollisionDispatch::sRegisterCollideShape(JoltCustomShapeSubType::VOXEL, JoltCustomShapeSubType::VOXEL, sCollideVoxelVsVoxel);
	JPH::CollisionDispatch::sRegisterCollideShape(JoltCustomShapeSubType::VOXEL, JoltCustomShapeSubType::VOXEL, JPH::CollisionDispatch::sReversedCollideShape);
}
