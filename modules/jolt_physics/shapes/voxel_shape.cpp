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

#include <Jolt/Geometry/RayAABox.h>

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

// NOTE: Unused right now, might be useful later for finding the surface voxel along a direction
JPH::Vec3 VoxelShape::FindSurfaceVoxel(JPH::Vec3 solidVoxelPos, JPH::Vec3 localPenetrationAxis) const
{
	// Convert the world-space penetration axis into a grid-space step direction.
	// The axis is already in shape2's local space, so we can map it to grid steps.
	// We step one voxel at a time along the dominant axis.
	JPH::Vec3 voxelSize = (mHalfExtents * 2.0f) / mResolution;

	// Convert local-space axis to grid-space axis (divide by voxel size)
	JPH::Vec3 gridStep = localPenetrationAxis / voxelSize;

	// Clamp to unit steps per axis — we move one voxel at a time
	gridStep = JPH::Vec3(
			gridStep.GetX() != 0.0f ? (gridStep.GetX() > 0.0f ? 1.0f : -1.0f) : 0.0f,
			gridStep.GetY() != 0.0f ? (gridStep.GetY() > 0.0f ? 1.0f : -1.0f) : 0.0f,
			gridStep.GetZ() != 0.0f ? (gridStep.GetZ() > 0.0f ? 1.0f : -1.0f) : 0.0f);

	// Only step along the dominant axis (matches how localNormal is computed)
	int axisIndex = localPenetrationAxis.Abs().GetHighestComponentIndex();
	JPH::Vec3 dominantStep = JPH::Vec3::sZero();
	dominantStep.SetComponent(axisIndex, gridStep[axisIndex]);

	JPH::Vec3 current = JPH::Vec3(
			std::floor(solidVoxelPos.GetX()),
			std::floor(solidVoxelPos.GetY()),
			std::floor(solidVoxelPos.GetZ()));

	// Walk outward (in the penetration direction) until we leave solid voxels
	// Cap iterations to the resolution to avoid infinite loops
	int maxSteps = (int)mResolution[axisIndex] + 1;
	for (int step = 0; step < maxSteps; ++step)
	{
		JPH::Vec3 next = current + dominantStep;
		if (!IsSolidAt(next))
			break; // current is the surface voxel

		current = next;
	}

	return current;
}

void VoxelShape::sCollidePointsVsGrid(
		const VoxelShape *shape1, const VoxelShape *shape2,
		JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
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

	// Corners that are in the voxel volume, in voxel grid space.
	const uint8_t *cornerDataShape1 = shape1->mVoxelCornerData;
	int numCornersShape1 = (int)shape1->mVoxelCornerDataSize / 4;

	// @todo(Voxel): This needs to be cleaned up.
		JPH::Mat44 inverse_transform1 = inCenterOfMassTransform1.InversedRotationTranslation();
		JPH::Mat44 transform_2_to_1 = inverse_transform1 * inCenterOfMassTransform2;

	for (int i = 0; i < numCornersShape1; i++)
	{
		// Position of the voxel in voxel grid space. Ranges from 0 to mResolution.axis
		int baseIndex = i * 4;
		JPH::Vec3 gridPosVoxelShape1((float)cornerDataShape1[baseIndex], (float)cornerDataShape1[baseIndex + 1], (float)cornerDataShape1[baseIndex + 2]);

		// Voxel grid space to local space for shape 1
		JPH::Vec3 posLocalShape1 = shape1->GetLocalPos(gridPosVoxelShape1);

		// Move the point into Shape 2 local space
		JPH::Vec3 posInShape2Local = transform1To2 * posLocalShape1;

		// Map Shape 2 local space to its internal voxel grid coordinates
		JPH::Vec3 posInGridVoxel = shape2->GetGridIndex(posInShape2Local);

		if (shape2->CheckVoxelCollision(posInGridVoxel))
		{
			JPH::Vec3 posLocalShape2 = shape2->GetLocalPos(posInGridVoxel);




			// Calculate the norml for the penetration
			// Normalize the position by the half-extents
			// This turns your rectangle/pancake into a virtual 1x1x1 cube
			JPH::Vec3 normalizedPos = posLocalShape2 / shape2->mHalfExtents;

			// @continue(Darren): Cleanup
			// The axis with the LARGEST normalized value is the face we are closest to
			// even on a thin object.
			int axisIndex = normalizedPos.Abs().GetHighestComponentIndex();
			float sign = normalizedPos[axisIndex] > 0.0f ? 1.0f : -1.0f;

			// @todo(Voxel): This needs to be cleaned up.
			// Build local penetration axis
			JPH::Vec3 localPenetrationAxis = JPH::Vec3::sZero();
			localPenetrationAxis.SetComponent(axisIndex, sign);

			// Find the primary direction of this vector
			int axisIndex_test = localPenetrationAxis.Abs().GetHighestComponentIndex();
			float sign_test = localPenetrationAxis[axisIndex_test] > 0.0f ? 1.0f : -1.0f;

			JPH::Vec3 localNormal = JPH::Vec3::sZero();
			localNormal.SetComponent(axisIndex_test, sign_test);





			// Offset the positions by half a voxel along the normal direction to get the surface contact point in local space.
			// @continue(Darren): Need to transform the local normal or calcualte for other shape, dont minus and plus, math should be the same.
			JPH::Vec3 pos1LocalSurface = posLocalShape1 - localNormal * (voxelSize1 * 0.5f);
			JPH::Vec3 pos2LocalSurface = posLocalShape2 + localNormal * (voxelSize2 * 0.5f);

			// Transform normal to World Space
			JPH::Vec3 worldNormal = inCenterOfMassTransform2.Multiply3x3(localNormal);
			JPH::Vec3 penetrationAxis = inIsShape1ProvidingPoints ? -worldNormal : worldNormal;

			JPH::Vec3 inContactPointOn1 = inCenterOfMassTransform1 * pos1LocalSurface;
			JPH::Vec3 inContactPointOn2 = inCenterOfMassTransform2 * pos2LocalSurface;

			float penetrationDepthMeters = (inContactPointOn1 - inContactPointOn2).Dot(penetrationAxis);

			// Check if the penetration is bigger than the early out fraction
			if (-penetrationDepthMeters < ioCollector.GetEarlyOutFraction())
			{
				JPH::CollideShapeResult result(
						inContactPointOn1, inContactPointOn2, penetrationAxis, penetrationDepthMeters,
						inSubShapeIDCreator1.GetID(), inSubShapeIDCreator2.GetID(),
						JPH::TransformedShape::sGetBodyID(ioCollector.GetContext()));

				// Gather faces
				if (inCollideShapeSettings.mCollectFacesMode == JPH::ECollectFacesMode::CollectFaces)
				{
					// Get supporting face of shape 1
					shape1->GetSupportingFace(JPH::SubShapeID(), -penetrationAxis, inScale1, inCenterOfMassTransform1, result.mShape1Face);

					// Get supporting face of shape 2
					shape2->GetSupportingFace(JPH::SubShapeID(), transform_2_to_1.Multiply3x3Transposed(penetrationAxis), inScale2, inCenterOfMassTransform2, result.mShape2Face);
				}

				ioCollector.AddHit(result);
			}
		}
	}
}

void VoxelShape::sCollideVoxelVsVoxelLocal(
		const VoxelShape *inShape1,
		const VoxelShape *inShape2,
		JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
		JPH::Mat44Arg inCenterOfMassTransform1,
		JPH::Mat44Arg inCenterOfMassTransform2,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator1,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
		const JPH::CollideShapeSettings &inCollideShapeSettings,
		JPH::CollideShapeCollector &ioCollector)
{
	// @todo(Voxel): Cleanup. Can calculate this inside sCollidePointsVsGrid
	// A's local points into B's local space
	JPH::Mat44 transform1To2 = inCenterOfMassTransform2.Inversed() * inCenterOfMassTransform1;

	// Shape 1 corners vs Shape 2 grid
	sCollidePointsVsGrid(
			inShape1, inShape2,
			inScale1, inScale2,
			transform1To2,
			inCenterOfMassTransform1, inCenterOfMassTransform2,
			inSubShapeIDCreator1, inSubShapeIDCreator2,
			true, inCollideShapeSettings, ioCollector);
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

	sCollideVoxelVsVoxelLocal(shape1, shape2, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector);
}

// For ray casting we just do it against the bounding box. We don't need to know about the voxel level here
bool VoxelShape::CastRay(const JPH::RayCast &inRay, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::RayCastResult &ioHit) const
{
	// Test hit against box
	float fraction = JPH::max(JPH::RayAABox(inRay.mOrigin, JPH::RayInvDirection(inRay.mDirection), -mHalfExtents, mHalfExtents), 0.0f);
	if (fraction < ioHit.mFraction)
	{
		ioHit.mFraction = fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}
	return false;
}

void VoxelShape::CastRay(const JPH::RayCast &inRay, const JPH::RayCastSettings &inRayCastSettings, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CastRayCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	float min_fraction, max_fraction;
	JPH::RayAABox(inRay.mOrigin, JPH::RayInvDirection(inRay.mDirection), -mHalfExtents, mHalfExtents, min_fraction, max_fraction);
	if (min_fraction <= max_fraction // Ray should intersect
			&& max_fraction >= 0.0f // End of ray should be inside box
			&& min_fraction < ioCollector.GetEarlyOutFraction()) // Start of ray should be before early out fraction
	{
		// Better hit than the current hit
		JPH::RayCastResult hit;
		hit.mBodyID = JPH::TransformedShape::sGetBodyID(ioCollector.GetContext());
		hit.mSubShapeID2 = inSubShapeIDCreator.GetID();

		// Check front side
		if (inRayCastSettings.mTreatConvexAsSolid || min_fraction > 0.0f)
		{
			hit.mFraction = JPH::max(0.0f, min_fraction);
			ioCollector.AddHit(hit);
		}

		// Check back side hit
		if (inRayCastSettings.mBackFaceModeConvex == JPH::EBackFaceMode::CollideWithBackFaces && max_fraction < ioCollector.GetEarlyOutFraction())
		{
			hit.mFraction = max_fraction;
			ioCollector.AddHit(hit);
		}
	}
}

JPH::MassProperties VoxelShape::GetMassProperties() const
{
	// Treat the inertia like it's a box, rigid bodies should have their bounding
	// volume be somewhat tight around the voxels
	JPH::MassProperties p;
	p.SetMassAndInertiaOfSolidBox(2.0f * mHalfExtents, GetDensity());

	return p;
}

JPH::Vec3 VoxelShape::GetSurfaceNormal(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inLocalSurfacePosition) const
{
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	// Get component that is closest to the surface of the box
	int index = (inLocalSurfacePosition.Abs() - mHalfExtents).Abs().GetLowestComponentIndex();

	// Calculate normal
	JPH::Vec3 normal = JPH::Vec3::sZero();
	normal.SetComponent(index, inLocalSurfacePosition[index] > 0.0f ? 1.0f : -1.0f);
	return normal;
}

void VoxelShape::GetSupportingFace(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inDirection, JPH::Vec3Arg inScale, JPH::Mat44Arg inCenterOfMassTransform, JPH::Shape::SupportingFace &outVertices) const
{
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	JPH::Vec3 scaled_half_extent = inScale.Abs() * mHalfExtents;
	JPH::AABox box(-scaled_half_extent, scaled_half_extent);
	box.GetSupportingFace(inDirection, outVertices);

	// Transform to world space
	for (JPH::Vec3 &v : outVertices) {
		v = inCenterOfMassTransform * v;
	}
}

const JPH::PhysicsMaterial* VoxelShape::GetMaterial(const JPH::SubShapeID &inSubShapeID) const
{
	return JPH::PhysicsMaterial::sDefault;
}

// Set density of the shape (kg / m^3)
void VoxelShape::SetDensity(float inDensity)
{
	mDensity = inDensity;
}

// Get density of the shape (kg / m^3)
float VoxelShape::GetDensity() const
{
	return mDensity;
}

void VoxelShape::sRegister()
{
	JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL);
	f.mConstruct = []() -> Shape * { return new VoxelShape; };
	f.mColor = JPH::Color::sOrange;

	JPH::CollisionDispatch::sRegisterCollideShape(JoltCustomShapeSubType::VOXEL, JoltCustomShapeSubType::VOXEL, sCollideVoxelVsVoxel);
}
