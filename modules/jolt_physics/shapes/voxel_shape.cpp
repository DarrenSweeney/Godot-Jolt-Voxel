#include "voxel_shape.h"

#include "jolt_custom_shape_type.h"
#include "../spaces/jolt_query_collectors.h"

#include "Jolt/Physics/Collision/CastResult.h"
#include "Jolt/Physics/Collision/RayCast.h"
#include "Jolt/Physics/Collision/TransformedShape.h"
#include "Jolt/Physics/Collision/CollidePointResult.h"
#include "Jolt/Physics/Collision/CollisionCollectorImpl.h"
#include "Jolt/Physics/Collision/Shape/DecoratedShape.h"


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
#if 0
	if (GetLocalBounds().Contains(inPoint) && mClassificationData)
	{
		JPH::Vec3 coord = GetVoxelCoord(inPoint);
		int index = GetIndex((uint32_t)coord.GetX(), (uint32_t)coord.GetY(), (uint32_t)coord.GetZ());

		if (mClassificationData[index] > 0)
		{
			JPH::CollidePointResult result;
			result.mSubShapeID2 = inSubShapeIDCreator.GetID();
			result.mBodyID = JPH::TransformedShape::sGetBodyID(ioCollector.GetContext());

			ioCollector.AddHit(result);
		}
	}
#endif
}

bool VoxelShape::IsSolidAt(JPH::Vec3Arg inLocalPoint) const
{
	// 1. Convert floating point local position to integer grid coordinates
	// Assuming your grid starts at (0,0,0) and each voxel is 1.0 units.
	// If your voxels are a different size, you'd divide by voxel_size first.
	int x = (int)std::floor(inLocalPoint.GetX());
	int y = (int)std::floor(inLocalPoint.GetY());
	int z = (int)std::floor(inLocalPoint.GetZ());

	// 2. Bounds check
	if (x < 0 || y < 0 || z < 0 || x >= mResolution.GetX() || y >= mResolution.GetY() || z >= mResolution.GetZ()) {
		return false;
	}

	// 3. Calculate the linear index of the voxel
	int64_t index = (int64_t)z * mResolution.GetY() * mResolution.GetX() + (int64_t)y * mResolution.GetX() + x;

	// 4. Extract the bit
	// index >> 3 is the same as index / 8 (finds the byte)
	// index & 7 is the same as index % 8 (finds the bit position 0-7)
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

// UNUSED!!!!!
#if 0
JPH::Vec3 VoxelShape::GetSurfaceNormalAt(JPH::Vec3Arg inLocalPoint) const
{
	JPH::Vec3 coord = GetVoxelCoord(inLocalPoint);
	int x = (int)coord.GetX();
	int y = (int)coord.GetY();
	int z = (int)coord.GetZ();

	JPH::Vec3 normal(0, 0, 0);
	// Check 6 neighbors; if a neighbor is empty, that face contributes to the normal
	if (x + 1 >= mResolution.GetX() || !IsSolidAt(GetLocalPos(x + 1, y, z))) 
		normal += JPH::Vec3(1, 0, 0);
	
	if (x - 1 < 0 || !IsSolidAt(GetLocalPos(x - 1, y, z))) 
		normal += JPH::Vec3(-1, 0, 0);
	
	if (y + 1 >= mResolution.GetY() || !IsSolidAt(GetLocalPos(x, y + 1, z))) 
		normal += JPH::Vec3(0, 1, 0);
	
	if (y - 1 < 0 || !IsSolidAt(GetLocalPos(x, y - 1, z))) 
		normal += JPH::Vec3(0, -1, 0);
	
	if (z + 1 >= mResolution.GetZ() || !IsSolidAt(GetLocalPos(x, y, z + 1))) 
		normal += JPH::Vec3(0, 0, 1);
	
	if (z - 1 < 0 || !IsSolidAt(GetLocalPos(x, y, z - 1))) 
		normal += JPH::Vec3(0, 0, -1);
	

	return normal.IsNearZero() ? JPH::Vec3(0, 1, 0) : normal.Normalized();
}
#endif


void VoxelShape::sCollideVoxelVsVoxelLocal(
		const VoxelShape *inShape1, // The shape we are testing points FROM
		const VoxelShape *inShape2, // The shape we are testing volume AGAINST
		JPH::Mat44Arg inCenterOfMassTransform1, // Transform for shape 1
		JPH::Mat44Arg inCenterOfMassTransform2, // Transform for shape 2
		const JPH::SubShapeIDCreator &inSubShapeIDCreator1,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
		const JPH::AABox &inIntersection, // The pre-calculated AABB intersection
		JPH::CollideShapeCollector &ioCollector, // The Jolt collector
		bool inFlip // TRUE if this is Pass 2
)
{
	const uint8_t *datasets[] = { inShape1->mVoxelCornerData, inShape1->mVoxelEdgeData };
	const size_t sizes[] = { inShape1->mVoxelCornerDataSize, inShape1->mVoxelEdgeDataSize };

	for (int d = 0; d < 2; d++) {
		const uint8_t *data = datasets[d];
		size_t size = sizes[d];
		if (!data || size == 0) {
			continue;
		}

		for (size_t i = 0; i + 3 < size; i += 4) {
			JPH::Vec3 localPos = inShape1->GetLocalPos(data[i], data[i + 1], data[i + 2]);
			JPH::Vec3 worldPos = inCenterOfMassTransform1 * localPos;

			if (!inIntersection.Contains(worldPos)) {
				continue;
			}

			JPH::Vec3 localNormal = inShape1->DecodeNormal(data[i + 3]);
			JPH::Vec3 worldNormal = inCenterOfMassTransform1.Multiply3x3(localNormal);

			JPH::CollideShapeResult result;

			// Handle SubShape IDs and Points based on flip status
			if (!inFlip)
			{
				result.mSubShapeID1 = inSubShapeIDCreator1.GetID();
				result.mSubShapeID2 = inSubShapeIDCreator2.GetID();
				result.mContactPointOn1 = worldPos;
				result.mContactPointOn2 = worldPos;
				result.mPenetrationAxis = -worldNormal; // Axis pointing from 2 to 1
			} else
			{
				// PASS 2: Shape 2 is providing the points, so it is "Shape 1" locally
				// but must be reported as "Shape 2" to the collector.
				result.mSubShapeID1 = inSubShapeIDCreator2.GetID();
				result.mSubShapeID2 = inSubShapeIDCreator1.GetID();
				result.mContactPointOn1 = worldPos;
				result.mContactPointOn2 = worldPos;
				result.mPenetrationAxis = worldNormal; // Flip the normal direction
			}

			result.mPenetrationDepth = 0.1f;

			result.mSubShapeID1 = inSubShapeIDCreator1.GetID();
			result.mSubShapeID2 = inSubShapeIDCreator2.GetID();

			ioCollector.AddHit(result);

			if (ioCollector.ShouldEarlyOut()) {
				return;
			}
		}
	}
}

void VoxelShape::sCollideVoxelVsVoxel(const JPH::Shape *inShape1, const JPH::Shape *inShape2, JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
		JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
		const JPH::CollideShapeSettings &inCollideShapeSettings, JPH::CollideShapeCollector &ioCollector,
		const JPH::ShapeFilter &inShapeFilter)
{
	// 1. Unwrap here once
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

	// 2. Shared AABB check
	JPH::AABox worldBounds1 = shape1->GetWorldSpaceBounds(inCenterOfMassTransform1, inScale1);
	JPH::AABox worldBounds2 = shape2->GetWorldSpaceBounds(inCenterOfMassTransform2, inScale2);
	JPH::AABox intersection = worldBounds1.Intersect(worldBounds2);
	if (!intersection.IsValid()) {
		return;
	}

	// Pass 1: Shape 1's points against Shape 2's volume
	sCollideVoxelVsVoxelLocal(shape1, shape2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, intersection, ioCollector, false);

	// Pass 2: Shape 2's points against Shape 1's volume (Note the 'true' for flipping)
	sCollideVoxelVsVoxelLocal(shape2, shape1, inCenterOfMassTransform2, inCenterOfMassTransform1, inSubShapeIDCreator2, inSubShapeIDCreator1, intersection, ioCollector, true);
}

#ifdef JPH_DEBUG_RENDERER
void VoxelShape::Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const {
#if 0
	// NOTE: To prevent lag, we only draw if the object isn't massive
	if (mClassificationData && mResolution.GetX() <= 64)
	{
		// Example: Draw a small dot for every 'Corner' voxel
		for (uint32_t y = 0; y < (uint32_t)mResolution.GetY(); ++y)
		{
			for (uint32_t z = 0; z < (uint32_t)mResolution.GetZ(); ++z)
			{
				for (uint32_t x = 0; x < (uint32_t)mResolution.GetX(); ++x)
				{
					uint8_t type = mClassificationData[GetIndex(x, y, z)];
					if (type == 1) // TYPE_CORNER
					{
						JPH::Vec3 local_pos = GetLocalPos(x, y, z);
						JPH::RVec3 world_pos = inCenterOfMassTransform * local_pos;

						// Draw a tiny sphere at each corner
						inRenderer->DrawMarker(world_pos, JPH::Color::sGreen, 0.05f);
					}
				}
			}
		}
	}
#endif
}
#endif

void VoxelShape::sRegister()
{
	JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL);
	f.mConstruct = []() -> Shape * { return new VoxelShape; };
	f.mColor = JPH::Color::sOrange;

	JPH::CollisionDispatch::sRegisterCollideShape(JoltCustomShapeSubType::VOXEL, JoltCustomShapeSubType::VOXEL, sCollideVoxelVsVoxel);
}
