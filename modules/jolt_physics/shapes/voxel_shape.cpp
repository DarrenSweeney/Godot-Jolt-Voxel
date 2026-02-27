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

}

int VoxelShape::GetIndex(uint32_t x, uint32_t y, uint32_t z) const
{
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

bool VoxelShape::CheckVoxelCollision(JPH::Vec3 &voxelGridPos) const
{
	// Identify the 8 neighbors voxels in the voxel
	// We floor/ceil the local coordinates to find the surrounding voxel indices
	int minX = (int)std::floor(voxelGridPos.GetX());
	int minY = (int)std::floor(voxelGridPos.GetY());
	int minZ = (int)std::floor(voxelGridPos.GetZ());

#if 0
	voxelGridPos = JPH::Vec3(minX, minY, minZ);
	if (IsSolidAt(voxelGridPos)) {
		return true;
	}

	return false;
#else
	for (int x = minX; x <= minX + 1; ++x)
	{
		for (int y = minY; y <= minY + 1; ++y)
		{
			for (int z = minZ; z <= minZ + 1; ++z)
			{
				voxelGridPos = JPH::Vec3(x, y, z);

				// Check if the voxel at this point is solid.
				if (IsSolidAt(voxelGridPos))
				{					
					return true;
				}
			}
		}
	}

	return false;
#endif
}

JPH::Vec3 VoxelShape::GetLocalPos(const JPH::Vec3 &argIndex) const
{
	JPH::Vec3 halfOffset(0.5f, 0.5f, 0.5f);
	JPH::Vec3 fullSize = mHalfExtents * 2.0f;

	// (Index + 0.5) / Res * FullSize - HalfExtent
	// This maps index 0 to the "minimum" corner and centers everything
	return ((argIndex + halfOffset) / mResolution) * fullSize - mHalfExtents;
}

JPH::Vec3 VoxelShape::GetGridIndex(const JPH::Vec3 &argLocalPos) const
{
#if 0
	// If the point is outside the half-extents, it can't be in the grid
	if (std::abs(argLocalPos.GetX()) > mHalfExtents.GetX() ||
			std::abs(argLocalPos.GetY()) > mHalfExtents.GetY() ||
			std::abs(argLocalPos.GetZ()) > mHalfExtents.GetZ()) {
		// Return a sentinel value (like -1) to indicate "Outside"
		return JPH::Vec3::sReplicate(-1.0f);
	}
#endif

	JPH::Vec3 fullSize = mHalfExtents * 2.0f;

	// Shift the centered local pos (-HE to +HE) to positive range (0 to FullSize)
	JPH::Vec3 shiftedPos = argLocalPos + mHalfExtents;

	JPH::Vec3 fractionalIndex = (shiftedPos / fullSize) * mResolution;

	JPH::Vec3 maxIndex = mResolution - JPH::Vec3::sReplicate(1.0f);
	JPH::Vec3 clampedIndex = JPH::Vec3::sMin(JPH::Vec3::sMax(JPH::Vec3::sZero(), fractionalIndex), maxIndex);

	return clampedIndex;
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
		const JPH::CollideShapeSettings &inCollideShapeSettings,
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

		// Voxel space to local space for shape 1
		JPH::Vec3 posLocal = shape1->GetLocalPos(posVoxel);

		// Move the point into Shape 2 local space
		JPH::Vec3 posInShape2Local = transform1To2 * posLocal;

		// Map Shape 2 local space to its internal voxel grid coordinates
		JPH::Vec3 posInGridVoxel = shape2->GetGridIndex(posInShape2Local);

		// Calculate the norml for the penetration
		// Normalize the position by the half-extents
		// This turns your rectangle/pancake into a virtual 1x1x1 cube
		JPH::Vec3 normalizedPos = posInShape2Local / shape2->mHalfExtents;

		// The axis with the LARGEST normalized value is the face we are closest to
		// even on a thin object.
		int axisIndex = normalizedPos.Abs().GetHighestComponentIndex();
		float sign = normalizedPos[axisIndex] > 0.0f ? 1.0f : -1.0f;

		// Build your local penetration axis
		JPH::Vec3 localPenetrationAxis = JPH::Vec3::sZero();
		localPenetrationAxis.SetComponent(axisIndex, sign);

		// Find the primary direction of this vector
		int axisIndex_test = localPenetrationAxis.Abs().GetHighestComponentIndex();
		float sign_test = localPenetrationAxis[axisIndex_test] > 0.0f ? 1.0f : -1.0f;

		JPH::Vec3 localNormal = JPH::Vec3::sZero();
		localNormal.SetComponent(axisIndex_test, sign_test);

		if (shape2->CheckVoxelCollision(posInGridVoxel))
		{
			JPH::Vec3 pos2Local = shape2->GetLocalPos(posInGridVoxel);

			// Offset the positions by half a voxel along the normal direction to get the surface contact point in local space.
			JPH::Vec3 pos1LocalSurface = posLocal -localNormal * (voxelSize1 * 0.5f);
			JPH::Vec3 pos2LocalSurface = pos2Local + localNormal * (voxelSize2 * 0.5f);

			// Transform normal to World Space
			JPH::Vec3 worldNormal = inCenterOfMassTransform2.Multiply3x3(localNormal);
			JPH::Vec3 penetrationAxis = inIsShape1ProvidingPoints ? -worldNormal : worldNormal;

			JPH::Vec3 inContactPointOn1 = inCenterOfMassTransform1 * pos1LocalSurface;
			JPH::Vec3 inContactPointOn2 = inCenterOfMassTransform2 * pos2LocalSurface;

			float penetrationDepthMeters = (inContactPointOn1 - inContactPointOn2).Dot(penetrationAxis);

			JPH::CollideShapeResult result(
					inContactPointOn1, inContactPointOn2, penetrationAxis, penetrationDepthMeters,
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
		const JPH::CollideShapeSettings &inCollideShapeSettings,
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
			true, inCollideShapeSettings, ioCollector);

#if 0
	// Side B: Shape 2 corners vs Shape 1 grid
	sCollidePointsVsGrid(
			inShape2, inShape1,
			transform2To1,
			inCenterOfMassTransform2, inCenterOfMassTransform1,
			inSubShapeIDCreator2, inSubShapeIDCreator1,
			false, inCollideShapeSettings, ioCollector);
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

	sCollideVoxelVsVoxelLocal(shape1, shape2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector);
}

JPH::MassProperties VoxelShape::GetMassProperties() const
{
	JPH::MassProperties p;
	p.mMass = 1.0f; // @todo(Voxel): Can we read this from godot?

	// Inertia for a solid box: (mass / 12) * (h^2 + d^2), etc.
	// We use full extents (half * 2)
	JPH::Vec3 size = mHalfExtents * 2.0f;
	float mass_factor = p.mMass / 12.0f;

	float x2 = size.GetX() * size.GetX();
	float y2 = size.GetY() * size.GetY();
	float z2 = size.GetZ() * size.GetZ();

	p.mInertia = JPH::Mat44::sZero();
	p.mInertia(0, 0) = mass_factor * (y2 + z2);
	p.mInertia(1, 1) = mass_factor * (x2 + z2);
	p.mInertia(2, 2) = mass_factor * (x2 + y2);
	p.mInertia(3, 3) = 1.0f;

	return p;
}

const JPH::PhysicsMaterial* VoxelShape::GetMaterial(const JPH::SubShapeID &inSubShapeID) const
{
	return JPH::PhysicsMaterial::sDefault;
}

void VoxelShape::sRegister()
{
	JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL);
	f.mConstruct = []() -> Shape * { return new VoxelShape; };
	f.mColor = JPH::Color::sOrange;

	JPH::CollisionDispatch::sRegisterCollideShape(JoltCustomShapeSubType::VOXEL, JoltCustomShapeSubType::VOXEL, sCollideVoxelVsVoxel);
}
