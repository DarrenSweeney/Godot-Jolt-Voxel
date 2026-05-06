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
#include <vector>

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
#if 0
	JPH::Vec3 fullSize = mHalfExtents * 2.0f;

	// Shift the centered local pos (-HE to +HE) to positive range (0 to FullSize)
	JPH::Vec3 shiftedPos = argLocalPos + mHalfExtents;

	JPH::Vec3 fractionalIndex = (shiftedPos / fullSize) * mResolution;

	JPH::Vec3 maxIndex = mResolution - JPH::Vec3::sReplicate(1.0f);
	JPH::Vec3 clampedIndex = JPH::Vec3::sMin(JPH::Vec3::sMax(JPH::Vec3::sZero(), fractionalIndex), maxIndex);

	return clampedIndex;
#endif

#if 1
	JPH::Vec3 fullSize = mHalfExtents * 2.0f;
	JPH::Vec3 shiftedPos = argLocalPos + mHalfExtents;
	JPH::Vec3 fractionalIndex = (shiftedPos / fullSize) * mResolution;

	// Reject before clamping — return -1 sentinel if outside
	if (fractionalIndex.GetX() < 0.0f || fractionalIndex.GetX() >= mResolution.GetX() ||
			fractionalIndex.GetY() < 0.0f || fractionalIndex.GetY() >= mResolution.GetY() ||
			fractionalIndex.GetZ() < 0.0f || fractionalIndex.GetZ() >= mResolution.GetZ()) {
		return JPH::Vec3::sReplicate(-1.0f); // sentinel
	}

	JPH::Vec3 maxIndex = mResolution - JPH::Vec3::sReplicate(1.0f);
	return JPH::Vec3::sMin(JPH::Vec3::sMax(JPH::Vec3::sZero(), fractionalIndex), maxIndex);
#endif
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

// Walk outward from an inside voxel in a CALLER-SPECIFIED direction.
// Used when the probe has sunk into shape2 — we walk backwards along the
// approach vector (shape1's normal rotated into shape2's space) to find the
// actual entry surface, which may be on a completely different face than the
// baked inside-voxel escape normal points to.
JPH::Vec3 VoxelShape::FindSurfaceVoxelInDirection(JPH::Vec3Arg inStartingGridPos, JPH::Vec3Arg inDir) const
{
	if (!IsSolidAt(inStartingGridPos))
		return inStartingGridPos;

	// Already a surface voxel? No walk needed.
	uint8_t startType;
	JPH::Vec3 dummy;
	GetVoxelMetadata(inStartingGridPos, startType, dummy);
	if (startType > VoxelType_Empty && startType < VoxelType_Inside)
		return inStartingGridPos;

	JPH::Vec3 dir = inDir.NormalizedOr(JPH::Vec3::sAxisY());

	JPH::Vec3 currentPos = inStartingGridPos;
	int maxSteps = (int)mResolution.Length() + 2;

	for (int i = 0; i < maxSteps; ++i)
	{
		currentPos += dir;

		JPH::Vec3 roundedPos(
				std::round(currentPos.GetX()),
				std::round(currentPos.GetY()),
				std::round(currentPos.GetZ()));

		// Out of bounds → last position before exiting was the surface
		if (roundedPos.GetX() < 0 || roundedPos.GetX() >= mResolution.GetX() ||
				roundedPos.GetY() < 0 || roundedPos.GetY() >= mResolution.GetY() ||
				roundedPos.GetZ() < 0 || roundedPos.GetZ() >= mResolution.GetZ())
		{
			// The voxel just before going OOB is our surface candidate
			JPH::Vec3 prevPos(
					std::round((currentPos.GetX() - dir.GetX())),
					std::round((currentPos.GetY() - dir.GetY())),
					std::round((currentPos.GetZ() - dir.GetZ())));
			return IsSolidAt(prevPos) ? prevPos : inStartingGridPos;
		}

		uint8_t type;
		JPH::Vec3 norm;
		GetVoxelMetadata(roundedPos, type, norm);

		// Hit a labelled surface voxel — perfect
		if (type > VoxelType_Empty && type < VoxelType_Inside)
			return roundedPos;

		// Walked into empty space → the previous solid step was the surface
		if (type == VoxelType_Empty)
		{
			JPH::Vec3 prevPos(
					std::round((currentPos.GetX() - dir.GetX())),
					std::round((currentPos.GetY() - dir.GetY())),
					std::round((currentPos.GetZ() - dir.GetZ())));
			if (IsSolidAt(prevPos))
				return prevPos;
			return inStartingGridPos;
		}
		// VoxelType_Inside → keep walking
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

	// --- Pass 1: collect all penetrating surface voxels ---
	// We gather every contact candidate before emitting any hits so we can compute
	// a single consensus normal shared by all contacts in this pair. Without this,
	// each contact uses its own per-voxel baked normal; when those normals disagree
	// (e.g. an edge voxel next to a face voxel on a slope) the solver receives
	// conflicting penetration axes every frame, producing spin and jitter.
	struct ContactCandidate {
		JPH::Vec3 gridPosShape1;
		JPH::Vec3 gridPosShape2;
		JPH::Vec3 posLocalShape1;
		JPH::Vec3 posLocalShape2;
		JPH::Vec3 localNormal; // baked normal in shape2 local space
		uint8_t   voxelType;
	};

	std::vector<ContactCandidate> candidates;

	JPH::Vec3 voxelHalfSize1 = shape1->GetVoxelSize() * 0.5f;
	JPH::Vec3 voxelHalfSize2 = shape2->GetVoxelSize() * 0.5f;

	// Probe both corner and edge voxels of shape1 against shape2's grid.
	// Corners handle 3-axis contacts; edges handle the 2-axis case (e.g. a box
	// corner sliding along a ledge edge, or two edges meeting at an angle).
	// The probe logic is identical: offset the test point to the outer face of
	// each surface voxel so contact is detected at first touch, not after sinking.
	struct SurfaceSet { const uint8_t *data; int count; };
	const SurfaceSet sets[2] = {
		{ shape1->mVoxelCornerData, (int)shape1->mVoxelCornerDataSize / 4 },
		{ shape1->mVoxelEdgeData,   (int)shape1->mVoxelEdgeDataSize   / 4 },
	};

	for (const SurfaceSet &set : sets)
	{
		for (int i = 0; i < set.count; i++)
		{
			int baseIndex = i * 4;
			JPH::Vec3 gridPosVoxelShape1(
					(float)set.data[baseIndex],
					(float)set.data[baseIndex + 1],
					(float)set.data[baseIndex + 2]);

			uint8_t type1; JPH::Vec3 normal1;
			shape1->GetVoxelMetadata(gridPosVoxelShape1, type1, normal1);
			JPH::Vec3 faceOffset1 = (type1 != VoxelType_Inside) ? normal1 * voxelHalfSize1 : JPH::Vec3::sZero();
			JPH::Vec3 posLocalShape1 = shape1->GetLocalPos(gridPosVoxelShape1) + faceOffset1;

			JPH::Vec3 posInShape2Local   = transform1To2 * posLocalShape1;
			JPH::Vec3 gridPosVoxelShape2 = shape2->GetGridIndex(posInShape2Local);

			if (shape2->CheckVoxelCollision(gridPosVoxelShape2))
			{
				ContactCandidate c;
				c.gridPosShape1  = gridPosVoxelShape1;
				c.gridPosShape2  = gridPosVoxelShape2;
				c.posLocalShape1 = posLocalShape1;
				shape2->GetVoxelMetadata(gridPosVoxelShape2, c.voxelType, c.localNormal);

				// If the probe landed inside an interior voxel, walk BACKWARDS along
				// the approach direction to find the actual entry surface on shape2.
				// Using shape2's baked inside-voxel escape normal is wrong here:
				// that normal points outward based on the voxel's topology at rest,
				// but the probe may have entered from a completely different face
				// (e.g. corner piercing a side face → baked normal points up, but
				// the entry is through the side → we'd push in the wrong direction).
				if (c.voxelType == VoxelType_Inside)
				{
					// shape1's outward normal in shape2's local space gives the approach
					// direction. Negate it to walk back toward the entry surface.
					JPH::Vec3 approachInShape2 = transform1To2.Multiply3x3(normal1).NormalizedOr(JPH::Vec3::sAxisY());
					JPH::Vec3 surfaceGrid = shape2->FindSurfaceVoxelInDirection(gridPosVoxelShape2, -approachInShape2);
					shape2->GetVoxelMetadata(surfaceGrid, c.voxelType, c.localNormal);
					c.gridPosShape2 = surfaceGrid;
				}

				c.posLocalShape2 = shape2->GetLocalPos(c.gridPosShape2) + c.localNormal * voxelHalfSize2;
				candidates.push_back(c);
			}
		}
	}

	if (candidates.empty())
		return;

	// --- Compute consensus normal ---
	// Weight by voxel type reliability: Face voxels have the most stable outward
	// normal; Edge and Corner are less reliable; Inside voxels carry an escape
	// direction rather than a true surface normal so they are down-weighted heavily.
	//   VoxelType_Empty=0, Corner=1, Edge=2, Face=3, Inside=4
	static const float kTypeWeights[5] = { 0.0f, 1.0f, 2.0f, 3.0f, 0.25f };

	JPH::Vec3 consensusWorldNormal = JPH::Vec3::sZero();
	float totalWeight = 0.0f;

	for (const ContactCandidate &c : candidates)
	{
		float weight = (c.voxelType < 5) ? kTypeWeights[c.voxelType] : 1.0f;
		JPH::Vec3 worldNormal = inCenterOfMassTransform2.Multiply3x3(c.localNormal).NormalizedOr(JPH::Vec3::sAxisY());
		consensusWorldNormal += worldNormal * weight;
		totalWeight += weight;
	}

	consensusWorldNormal = (totalWeight > 0.0f)
			? (consensusWorldNormal / totalWeight).NormalizedOr(JPH::Vec3::sAxisY())
			: JPH::Vec3::sAxisY();

	// --- Pass 2: emit contacts using the consensus normal ---
	auto PackVoxelID = [](const JPH::SubShapeIDCreator &inBaseCreator, JPH::Vec3Arg inGridPos) -> JPH::SubShapeID {
		JPH::SubShapeIDCreator creator = inBaseCreator;
		creator.PushID((JPH::uint)inGridPos.GetX(), 10);
		creator.PushID((JPH::uint)inGridPos.GetY(), 10);
		creator.PushID((JPH::uint)inGridPos.GetZ(), 10);
		return creator.GetID();
	};

	for (const ContactCandidate &c : candidates)
	{
		JPH::Vec3 contactPointOn1 = inCenterOfMassTransform1 * c.posLocalShape1;
		JPH::Vec3 contactPointOn2 = inCenterOfMassTransform2 * c.posLocalShape2;

		// diff points from shape2's contact surface INTO shape2 (probe is inside shape2,
		// contact point is on shape2's surface above/outside it). That makes diff point
		// opposite to the outward consensusWorldNormal, so diff.Dot(normal) is negative.
		// Negate to get a positive penetration depth that scales with actual overlap.
		JPH::Vec3 diff = contactPointOn1 - contactPointOn2;
		float penetrationDepth = JPH::max(0.001f, -diff.Dot(consensusWorldNormal));

		if (-penetrationDepth < ioCollector.GetEarlyOutFraction())
		{
			JPH::SubShapeID id1 = PackVoxelID(inSubShapeIDCreator1, c.gridPosShape1);
			JPH::SubShapeID id2 = PackVoxelID(inSubShapeIDCreator2, c.gridPosShape2);

			JPH::CollideShapeResult result(
					contactPointOn1,
					contactPointOn2,
					-consensusWorldNormal,
					penetrationDepth,
					id1, id2,
					JPH::TransformedShape::sGetBodyID(ioCollector.GetContext()));

			if (inCollideShapeSettings.mCollectFacesMode == JPH::ECollectFacesMode::CollectFaces)
			{
				JPH::Vec3 surfaceGridPos1     = shape1->FindSurfaceVoxelAlongNormal(c.gridPosShape1);
				JPH::Vec3 surfacePosLocal1    = shape1->GetLocalPos(surfaceGridPos1);

				JPH::Vec3 surfaceGridPos2     = shape2->FindSurfaceVoxelAlongNormal(c.gridPosShape2);
				JPH::Vec3 surfacePosLocal2    = shape2->GetLocalPos(surfaceGridPos2);

				shape1->GetSupportingFace(id1,
						inCenterOfMassTransform1.Multiply3x3Transposed(consensusWorldNormal),
						inScale1, inCenterOfMassTransform1,
						result.mShape1Face, surfacePosLocal1, contactPointOn1);

				shape2->GetSupportingFace(id2,
						inCenterOfMassTransform2.Multiply3x3Transposed(-consensusWorldNormal),
						inScale2, inCenterOfMassTransform2,
						result.mShape2Face, surfacePosLocal2, contactPointOn2);
			}

			ioCollector.AddHit(result);
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
	// Pass A: Shape 1 surface voxels probe shape 2's grid
	int hitsBefore = ioCollector.GetNumHits();
	sCollidePointsVsGrid(
			inShape1, inShape2,
			inScale1, inScale2,
			inCenterOfMassTransform1, inCenterOfMassTransform2,
			inSubShapeIDCreator1, inSubShapeIDCreator2,
			inCollideShapeSettings, ioCollector);

	// Pass B: Shape 2 surface voxels probe shape 1's grid.
	// Only run if pass A found nothing — this handles the case where shape2's
	// corners are fully inside shape1 (so shape1→shape2 probing finds no hits)
	// but would double the separation force if both passes fire simultaneously.
	if (ioCollector.GetNumHits() == hitsBefore)
	{
		sCollidePointsVsGrid(
				inShape2, inShape1,
				inScale2, inScale1,
				inCenterOfMassTransform2, inCenterOfMassTransform1,
				inSubShapeIDCreator2, inSubShapeIDCreator1,
				inCollideShapeSettings, ioCollector);
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

	JPH::Vec3 voxelScaledHalfExtent = inScale.Abs() * (GetVoxelSize() * 0.5f);

	// Expand the face in the two axes perpendicular to the contact normal.
	// A single-voxel face forces the manifold builder to clip a tiny quad against
	// the opposing shape face, which breaks down on slopes where adjacent contacts
	// sit on different voxels. Expanding by 1.5x lets the faces from neighbouring
	// contacts overlap so the builder can assemble a proper multi-point manifold.
	// The normal axis keeps single-voxel thickness — we don't want false depth.
	int normalAxis = inDirection.Abs().GetHighestComponentIndex();
	const float kPerp = 1.5f;

	JPH::Vec3 faceHalfExtent(
			(normalAxis == 0) ? voxelScaledHalfExtent.GetX() : voxelScaledHalfExtent.GetX() * kPerp,
			(normalAxis == 1) ? voxelScaledHalfExtent.GetY() : voxelScaledHalfExtent.GetY() * kPerp,
			(normalAxis == 2) ? voxelScaledHalfExtent.GetZ() : voxelScaledHalfExtent.GetZ() * kPerp);

	JPH::AABox voxelBox(localContactPoint - faceHalfExtent, localContactPoint + faceHalfExtent);
	voxelBox.GetSupportingFace(inDirection, outVertices);

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
