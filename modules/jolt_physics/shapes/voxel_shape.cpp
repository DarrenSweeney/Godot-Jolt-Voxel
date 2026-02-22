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
	// Identify the 8 neighbors voxels in the voxel
	// We floor/ceil the local coordinates to find the surrounding voxel indices
	int minX = (int)std::floor(voxelGridPos.GetX());
	int minY = (int)std::floor(voxelGridPos.GetY());
	int minZ = (int)std::floor(voxelGridPos.GetZ());

	const uint8_t* voxelGrid = mVoxelBitfieldData;
	const size_t voxelGridSize = mVoxelBitfieldSize;

	for (int x = minX; x <= minX + 1; ++x)
	{
		for (int y = minY; y <= minY + 1; ++y)
		{
			for (int z = minZ; z <= minZ + 1; ++z)
			{
				const JPH::Vec3 pos = JPH::Vec3(x, y, z);

				// Check if the voxel at this point is solid.
				if (IsSolidAt(pos))
					return true;
			}
		}
	}

	return false;
}

void VoxelShape::sCollidePointsVsGrid(
		const VoxelShape *shape1,
		const VoxelShape *shape2,
		JPH::Mat44Arg transform1To2,
		JPH::Mat44Arg inCenterOfMassTransform1,
		JPH::Mat44Arg inCenterOfMassTransform2,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator1,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
		bool inIsShape1ProvidingPoints, // Logic flip to handle normal direction
		JPH::CollideShapeCollector &ioCollector)
{
	// Derive voxel scaling from the actual shape dimensions
	JPH::Vec3 voxelSize1 = (shape1->mHalfExtents * 2.0f) / shape1->mResolution;
	JPH::Vec3 voxelSize2 = (shape2->mHalfExtents * 2.0f) / shape2->mResolution;

	// This is the conversion factor for the target grid
	JPH::Vec3 invVoxelSize2 = JPH::Vec3(1.0f / voxelSize2.GetX(), 1.0f / voxelSize2.GetY(), 1.0f / voxelSize2.GetZ());

	// Corners that are in the voxel volume, in voxel grid space.
	const uint8_t *corner_data = shape1->mVoxelCornerData;
	int num_corners = (int)shape1->mVoxelCornerDataSize / 4;

	for (int i = 0; i < num_corners; i++)
	{
		// Position of the voxel in voxel grid space. Ranges from 0 to mResolution.axis
		JPH::Vec3 posVoxel((float)corner_data[i * 4 + 0], (float)corner_data[i * 4 + 1], (float)corner_data[i * 4 + 2]);

		// Map corner index to Shape 1 local space
		JPH::Vec3 posLocal = (posVoxel * voxelSize1) - shape1->mHalfExtents;

		// Move the point into Shape 2 local space
		JPH::Vec3 posInGridLocal = transform1To2 * posLocal;

		// Map Shape 2 local space to its internal voxel grid coordinates
		JPH::Vec3 posInGridVoxel = (posInGridLocal + shape2->mHalfExtents) * invVoxelSize2;

		if (shape2->CheckVoxelCollision(posInGridVoxel))
		{
			JPH::Vec3 voxelCenterVoxel(
					std::floor(posInGridVoxel.GetX()) + 0.5f,
					std::floor(posInGridVoxel.GetY()) + 0.5f,
					std::floor(posInGridVoxel.GetZ()) + 0.5f);

			// Normal and Penetration calculation using Shape 2's scale
			JPH::Vec3 offsetFromVoxelCenter = posInGridVoxel - voxelCenterVoxel;
			JPH::Vec3 localNormal = offsetFromVoxelCenter.Normalized();

			// Transform normal to World Space
			JPH::Vec3 worldNormal = inCenterOfMassTransform2.Multiply3x3(localNormal);
			JPH::Vec3 finalNormal = inIsShape1ProvidingPoints ? -worldNormal : worldNormal;

			JPH::Vec3 worldPos = inCenterOfMassTransform1 * posLocal;

			// Calculate depth in meters based on Shape 2's voxel dimensions
			// We project the offset onto the normal and scale by voxel size
			float distFromCenterVoxel = offsetFromVoxelCenter.Length();
			float penetrationDepthMeters = 5.0f;
			//std::max(0.0f, (0.5f - distFromCenterVoxel) * voxelSize2.Length() / std::sqrt(3.0f));
			finalNormal = JPH::Vec3(0, -1, 0);

			JPH::CollideShapeResult result(
					worldPos, worldPos, finalNormal, penetrationDepthMeters,
					inSubShapeIDCreator1.GetID(), inSubShapeIDCreator2.GetID(),
					JPH::TransformedShape::sGetBodyID(ioCollector.GetContext()));

			ioCollector.AddHit(result);
		}
	}
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
	// A's local points into B's local space
	JPH::Mat44 transform1To2 = inCenterOfMassTransform2.Inversed() * inCenterOfMassTransform1;

	// B's local points into A's local space
	JPH::Mat44 transform2To1 = inCenterOfMassTransform1.Inversed() * inCenterOfMassTransform2;

	// Side A: Shape 1 corners vs Shape 2 grid
	sCollidePointsVsGrid(
			inShape1, inShape2,
			transform1To2,
			inCenterOfMassTransform1, inCenterOfMassTransform2,
			inSubShapeIDCreator1, inSubShapeIDCreator2,
			true, ioCollector);

#if 0
	// Side B: Shape 2 corners vs Shape 1 grid
	sCollidePointsVsGrid(
			inShape2, inShape1,
			transform2To1,
			inCenterOfMassTransform2, inCenterOfMassTransform1,
			inSubShapeIDCreator2, inSubShapeIDCreator1,
			false, ioCollector);
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
	if (!shape1 || !shape2)
		return;

	// AABB check
	JPH::AABox worldBounds1 = shape1->GetWorldSpaceBounds(inCenterOfMassTransform1, inScale1);
	JPH::AABox worldBounds2 = shape2->GetWorldSpaceBounds(inCenterOfMassTransform2, inScale2);
	JPH::AABox intersection = worldBounds1.Intersect(worldBounds2);
	if (!intersection.IsValid()) 
		return;

	sCollideVoxelVsVoxelLocal(shape1, shape2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, intersection, ioCollector);
}

void VoxelShape::sRegister()
{
	JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL);
	f.mConstruct = []() -> Shape * { return new VoxelShape; };
	f.mColor = JPH::Color::sOrange;

	JPH::CollisionDispatch::sRegisterCollideShape(JoltCustomShapeSubType::VOXEL, JoltCustomShapeSubType::VOXEL, sCollideVoxelVsVoxel);
}
