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

#if 1
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
	JPH::Vec3 fullSize = mHalfExtents * 2.0f;

	// Shift the centered local pos (-HE to +HE) to positive range (0 to FullSize)
	JPH::Vec3 shiftedPos = argLocalPos + mHalfExtents;

	JPH::Vec3 fractionalIndex = (shiftedPos / fullSize) * mResolution;

	JPH::Vec3 maxIndex = mResolution - JPH::Vec3::sReplicate(1.0f);
	JPH::Vec3 clampedIndex = JPH::Vec3::sMin(JPH::Vec3::sMax(JPH::Vec3::sZero(), fractionalIndex), maxIndex);

	return clampedIndex;
}

// @todo: Maybe use pre-baked data instead?
JPH::Vec3 VoxelShape::FindSurfaceVoxel(JPH::Vec3 solidVoxelPos) const {
	if (!IsSolidAt(solidVoxelPos)) {
		return solidVoxelPos;
	}

	auto IsSurfaceVoxel = [&](const JPH::Vec3 &pos) -> bool {
		if (!IsSolidAt(pos)) {
			return false;
		}

		static const JPH::Vec3 directions[] = {
			JPH::Vec3(1, 0, 0), JPH::Vec3(-1, 0, 0),
			JPH::Vec3(0, 1, 0), JPH::Vec3(0, -1, 0),
			JPH::Vec3(0, 0, 1), JPH::Vec3(0, 0, -1)
		};

		for (const JPH::Vec3 &dir : directions) {
			if (!IsSolidAt(pos + dir)) {
				return true;
			}
		}
		return false;
	};

	if (IsSurfaceVoxel(solidVoxelPos)) {
		return solidVoxelPos;
	}

	// Expand a cubic shell at each step, checking all voxels on the shell surface
	int maxSteps = 8;
	for (int i = 1; i < maxSteps; i++) {
		for (int x = -i; x <= i; x++) {
			for (int y = -i; y <= i; y++) {
				for (int z = -i; z <= i; z++) {
					// Only check voxels on the outer shell (not inner shells already checked)
					if (abs(x) != i && abs(y) != i && abs(z) != i) {
						continue;
					}

					JPH::Vec3 candidate = solidVoxelPos + JPH::Vec3((float)x, (float)y, (float)z);

					if (IsSurfaceVoxel(candidate)) {
						return candidate;
					}
				}
			}
		}
	}

	return solidVoxelPos;
}

JPH::Vec3 VoxelShape::FindSurfaceVoxelAlongNormal(JPH::Vec3Arg inStartingGridPos) const {
	// Check if the starting position is solid
	if (!IsSolidAt(inStartingGridPos)) {
		return inStartingGridPos;
	}

	uint8_t startType;
	JPH::Vec3 localNormal;
	GetVoxelMetadata(inStartingGridPos, startType, localNormal);

	// If the starting position is already a surface voxel (Corner, Edge, or Face), we are done
	if (startType > 0 && startType < 4) {
		return inStartingGridPos;
	}

	// If the normal is nearly zero, we cannot determine a direction to walk
	if (localNormal.IsNearZero()) {
		return inStartingGridPos;
	}

	// Normalize to ensure consistent step sizing
	localNormal = localNormal.Normalized();

	JPH::Vec3 currentPos = inStartingGridPos;
	int maxSteps = (int)mResolution.Length();

	for (int i = 0; i < maxSteps; ++i) {
		// Advance position along the stored surface normal
		currentPos += localNormal;

		// Convert to discrete grid coordinates
		JPH::Vec3 roundedPos(std::round(currentPos.GetX()),
				std::round(currentPos.GetY()),
				std::round(currentPos.GetZ()));

		// Bounds check against grid dimensions
		if (roundedPos.GetX() < 0 || roundedPos.GetX() >= mResolution.GetX() ||
				roundedPos.GetY() < 0 || roundedPos.GetY() >= mResolution.GetY() ||
				roundedPos.GetZ() < 0 || roundedPos.GetZ() >= mResolution.GetZ()) {
			return roundedPos;
		}

		uint8_t type;
		JPH::Vec3 norm;
		GetVoxelMetadata(roundedPos, type, norm);

		// Return if we hit a designated surface type
		if (type > 0 && type < 4) {
			return roundedPos;
		}

		// If we transition into empty space, return the last known solid position
		if (type == 0) {
			return roundedPos - localNormal;
		}
	}

	return inStartingGridPos;
}

void VoxelShape::GetVoxelMetadata(JPH::Vec3Arg inGridPos, uint8_t &outType, JPH::Vec3 &outNormal) const
{
	int index = GetIndex((uint32_t)inGridPos.GetX(), (uint32_t)inGridPos.GetY(), (uint32_t)inGridPos.GetZ());

	// Use the flat array (mVoxelData) not the bitfield
	uint8_t packedByte = mVoxelData[index];

	outType = packedByte >> 5;
	uint8_t normalIdx = packedByte & 0x1F;

	if (normalIdx < 26)
	{
		outNormal = normal_lut[normalIdx]; // Use your LUT of JPH::Vec3
	}
	else
	{
		outNormal = JPH::Vec3::sAxisY();
	}
}

void VoxelShape::sCollidePointsVsGrid(
		const VoxelShape *shape1, const VoxelShape *shape2,
		JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
		JPH::Mat44Arg inCenterOfMassTransform1,
		JPH::Mat44Arg inCenterOfMassTransform2,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator1,
		const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
		const JPH::CollideShapeSettings &inCollideShapeSettings,
		JPH::CollideShapeCollector &ioCollector)
{
	JPH::Mat44 transform1To2 = inCenterOfMassTransform2.Inversed() * inCenterOfMassTransform1;

	// Corners that are in the voxel volume, in voxel grid space.
	const uint8_t *cornerDataShape1 = shape1->mVoxelCornerData;
	int numCornersShape1 = (int)shape1->mVoxelCornerDataSize / 4;

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
		JPH::Vec3 gridPosVoxelShape2 = shape2->GetGridIndex(posInShape2Local);

		if (shape2->CheckVoxelCollision(gridPosVoxelShape2))
		{
			/**
			 * @brief The 'localNormal' is a pre-calculated unit vector stored in the voxel grid.
			 * * Unlike standard physics which calculates a normal based on the separation vector
			 * between two points (prone to jitter on voxel faces), this normal is fetched from
			 * a Look-Up Table (LUT) based on the voxel's pre-baked classification.
			 * * Logic by VoxelType:
			 * - SURFACE (Face, Edge, Corner): The average outward-facing normal of the voxel geometry.
			 * - INSIDE: A 'Closest Point of Exit' vector pointing toward the nearest empty cell.
			 * * PURPOSE: It provides a stable penetration axis for the Jolt solver, preventing
			 * 'force-flipping' when objects slide across voxel boundaries and eliminating
			 * the need for expensive runtime surface searches.
			 * * @note This vector is in Voxel-Local space and must be transformed by the
			 * body's Center of Mass (COM) rotation (Multiply3x3) before being used in world-space physics.
			 */
			uint8_t voxelType;
			JPH::Vec3 localNormal;
			shape2->GetVoxelMetadata(gridPosVoxelShape2, voxelType, localNormal);

			// 1. Convert the pre-baked local normal of Shape 2 into World Space
			// This is the stable "Out" direction of Shape 2
			JPH::Vec3 worldNormalShape2 = inCenterOfMassTransform2.Multiply3x3(localNormal).Normalized();

			// 2. The penetration axis Jolt expects points from Shape 2 towards Shape 1
			// Since localNormal points OUT of Shape 2, worldNormalShape2 is our axis
			JPH::Vec3 penetrationAxis = worldNormalShape2;

			JPH::Vec3 posLocalShape2 = shape2->GetLocalPos(gridPosVoxelShape2);

			// 3. Calculate penetration depth
			float penetrationDepth;
			if (voxelType == 4) // VoxelType::INSIDE
			{
				// For deep voxels, use the blog's trick: push out by a full voxel size
				// This ensures the object is aggressively ejected
				penetrationDepth = shape2->GetVoxelSize().GetX();
			} else {
				// For SURFACE voxels, project the actual distance onto the normal
				// This gives a much smoother "glide" over the surface
				JPH::Vec3 voxelCenterWorldShape1 = inCenterOfMassTransform1 * posLocalShape1;
				JPH::Vec3 voxelCenterWorldShape2 = inCenterOfMassTransform2 * posLocalShape2;

				JPH::Vec3 diff = voxelCenterWorldShape1 - voxelCenterWorldShape2;
				penetrationDepth = diff.Dot(worldNormalShape2);

				// Ensure we don't have a negative depth due to floating point noise
				penetrationDepth = JPH::max(0.001f, penetrationDepth);
			}

			// Check if the penetration is bigger than the early out fraction
			if (-penetrationDepth < ioCollector.GetEarlyOutFraction())
			{
				JPH::Vec3 contactPointOn1 = inCenterOfMassTransform1 * posLocalShape1;
				JPH::Vec3 contactPointOn2 = inCenterOfMassTransform2 * shape2->GetLocalPos(gridPosVoxelShape2);

				JPH::CollideShapeResult result(
						contactPointOn1,
						contactPointOn2,
						-penetrationAxis,
						penetrationDepth,
						inSubShapeIDCreator1.GetID(), inSubShapeIDCreator2.GetID(),
						JPH::TransformedShape::sGetBodyID(ioCollector.GetContext()));

				// Gather faces
				if (inCollideShapeSettings.mCollectFacesMode == JPH::ECollectFacesMode::CollectFaces)
				{
					// Traverse the grid to find where this 'Exit Normal' actually hits the air
					JPH::Vec3 surfaceGridPos1 = shape1->FindSurfaceVoxelAlongNormal(gridPosVoxelShape1);
					JPH::Vec3 surfacePosLocalShape1 = shape1->GetLocalPos(surfaceGridPos1);

					JPH::Vec3 surfaceGridPos2 = shape2->FindSurfaceVoxelAlongNormal(gridPosVoxelShape2);
					JPH::Vec3 surfacePosLocalShape2 = shape2->GetLocalPos(surfaceGridPos2);

					// Get supporting face of shape 1
					shape1->GetSupportingFace(JPH::SubShapeID(), inCenterOfMassTransform1.Multiply3x3Transposed(penetrationAxis), inScale1, inCenterOfMassTransform1, result.mShape1Face, surfacePosLocalShape1, contactPointOn1);

					// Get supporting face of shape 2
					shape2->GetSupportingFace(JPH::SubShapeID(), inCenterOfMassTransform2.Multiply3x3Transposed(-penetrationAxis), inScale2, inCenterOfMassTransform2, result.mShape2Face, surfacePosLocalShape2, contactPointOn2);
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
	// Shape 1 corners vs Shape 2 grid
	sCollidePointsVsGrid(
			inShape1, inShape2,
			inScale1, inScale2,
			inCenterOfMassTransform1, inCenterOfMassTransform2,
			inSubShapeIDCreator1, inSubShapeIDCreator2,
			inCollideShapeSettings, ioCollector);
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

void VoxelShape::GetSupportingFace(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inDirection, JPH::Vec3Arg inScale,
		JPH::Mat44Arg inCenterOfMassTransform, JPH::Shape::SupportingFace &outVertices, JPH::Vec3 localContactPoint, JPH::Vec3 worldContactPoint) const
{
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

#if 1
	// Get the unscaled size of a single voxel
	JPH::Vec3 voxelHalfExtent = GetVoxelSize() * 0.5f;

	// Apply scale to the half extent
	JPH::Vec3 voxelScaledHalfExtent = inScale.Abs() * voxelHalfExtent;

	// Create a temporary AABox representing just this one voxel's volume at origin
	JPH::AABox voxelBox(-voxelScaledHalfExtent, voxelScaledHalfExtent);

	// Get the supporting face for this tiny box (returns 4 vertices for a quad)
	voxelBox.GetSupportingFace(inDirection, outVertices);

	// Map those tiny face vertices to the correct spot
	for (JPH::Vec3 &v : outVertices) {
		// 1. Move the vertex to the voxel's specific local position
		// 2. Transform the local position to world space
		v = inCenterOfMassTransform * (v + localContactPoint);
	}
#endif

#if 0
	JPH::Vec3 scaled_half_extent = (inScale.Abs() * mHalfExtents);//*0.5;
	JPH::AABox box(-scaled_half_extent, scaled_half_extent);
	box.GetSupportingFace(inDirection, outVertices);

	// Transform to world space
	for (JPH::Vec3 &v : outVertices) {
		v = inCenterOfMassTransform * v;
	}
#endif
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
