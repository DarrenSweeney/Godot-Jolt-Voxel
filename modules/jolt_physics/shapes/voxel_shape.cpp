#include "voxel_shape.h"

#include "jolt_custom_shape_type.h"
#include "../spaces/jolt_query_collectors.h"

#include "Jolt/Physics/Collision/CastResult.h"
#include "Jolt/Physics/Collision/RayCast.h"
#include "Jolt/Physics/Collision/TransformedShape.h"
#include "Jolt/Physics/Collision/CollidePointResult.h"
#include "Jolt/Physics/Collision/CollisionCollectorImpl.h"


// --- VoxelShapeSettings -- 
JPH::ShapeSettings::ShapeResult VoxelShapeSettings::Create() const {
	if (mCachedResult.IsEmpty()) {
		new VoxelShape(*this, mCachedResult);
		print_line("VOXEL VoxelShapeSettings::Create");
	}

	return mCachedResult;
}

// --- VoxelShape ---
void VoxelShape::CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator &inSubShapeIDCreator,
		JPH::CollidePointCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter) const
{
	print_line("VOXEL CollidePoint");
	if (GetLocalBounds().Contains(inPoint) && mClassificationData)
	{
		JPH::Vec3 coord = GetVoxelCoord(inPoint);
		// Note: Ensure your GetIndex accepts floats or cast them here
		int index = GetIndex((uint32_t)coord.GetX(), (uint32_t)coord.GetY(), (uint32_t)coord.GetZ());

		if (mClassificationData[index] > 0) {
			// Explicitly create the result object to avoid initializer list ambiguity
			JPH::CollidePointResult result;
			result.mSubShapeID2 = inSubShapeIDCreator.GetID();
			result.mBodyID = JPH::TransformedShape::sGetBodyID(ioCollector.GetContext());

			ioCollector.AddHit(result);
		}
	}
}

void VoxelShape::GetTrianglesStart(GetTrianglesContext &ioContext, const JPH::AABox &inBox, JPH::Vec3Arg inPositionCOM,
									JPH::QuatArg inRotation, JPH::Vec3Arg inScale) const
{
	// We don't need a complex context yet, but Jolt needs to know we are starting
	//new (&ioContext) GetTrianglesContext();
}

int VoxelShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, JPH::Float3 *outTriangleVertices,
									const JPH::PhysicsMaterial **outMaterials) const {
	// For now, return 0 triangles but stay "safe".
	// If you want to see collisions, you'd iterate voxels in the inBox here
	// and output 12 triangles per solid voxel.
	return 0;
}

bool VoxelShape::IsSolidAt(JPH::Vec3Arg inLocalPoint) const {
	if (!GetLocalBounds().Contains(inLocalPoint) || !mClassificationData) {
		return false;
	}
	JPH::Vec3 coord = GetVoxelCoord(inLocalPoint);
	int index = GetIndex((uint32_t)coord.GetX(), (uint32_t)coord.GetY(), (uint32_t)coord.GetZ());
	return mClassificationData[index] > 0;
}

void VoxelShape::sCollideVoxelVsVoxel(const JPH::Shape *inShape1, const JPH::Shape *inShape2, JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2, JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2, const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2, const JPH::CollideShapeSettings &inCollideShapeSettings, JPH::CollideShapeCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter) {
	const VoxelShape *shape1 = static_cast<const VoxelShape *>(inShape1);
	const VoxelShape *shape2 = static_cast<const VoxelShape *>(inShape2);

	// 1. Get the overlapping AABB in World Space
	JPH::AABox worldBounds1 = shape1->GetWorldSpaceBounds(inCenterOfMassTransform1, inScale1);
	JPH::AABox worldBounds2 = shape2->GetWorldSpaceBounds(inCenterOfMassTransform2, inScale2);
	JPH::AABox intersection = worldBounds1.Intersect(worldBounds2);

	if (!intersection.IsValid()) {
		return;
	}

	// 2. Transform intersection to Local Space of Shape 1
	JPH::Mat44 inv1 = inCenterOfMassTransform1.Inversed();
	JPH::AABox localIntersect = intersection.Transformed(inv1);

	// 3. Loop through voxels in Shape 1 that are in the intersection
	JPH::Vec3 start = shape1->GetVoxelCoord(localIntersect.mMin);
	JPH::Vec3 end = shape1->GetVoxelCoord(localIntersect.mMax);

	for (uint32_t z = start.GetZ(); z <= end.GetZ(); ++z) {
		for (uint32_t y = start.GetY(); y <= end.GetY(); ++y) {
			for (uint32_t x = start.GetX(); x <= end.GetX(); ++x) {
				// Only check classification types 1 (Corner) or 2 (Edge)
				uint8_t type = shape1->mClassificationData[shape1->GetIndex(x, y, z)];
				if (type == 1 || type == 2) {
					JPH::Vec3 localPos = shape1->GetLocalPos(x, y, z);
					JPH::Vec3 worldPos = inCenterOfMassTransform1 * localPos;

					// 4. Transform point into Shape 2's local space
					JPH::Vec3 localTo2 = inCenterOfMassTransform2.Inversed() * worldPos;

					// 5. Check if Shape 2 is solid there
					if (shape2->IsSolidAt(localTo2)) {
						// Create a collision result
						JPH::CollideShapeResult result;
						result.mContactPointOn1 = worldPos;
						result.mContactPointOn2 = worldPos;
						result.mPenetrationAxis = (worldPos - inCenterOfMassTransform1.GetTranslation()).Normalized(); // Placeholder normal
						result.mPenetrationDepth = 0.1f; // You'll want to calculate actual depth later
						result.mSubShapeID1 = inSubShapeIDCreator1.GetID();
						result.mSubShapeID2 = inSubShapeIDCreator2.GetID();

						ioCollector.AddHit(result);
					}
				}
			}
		}
	}
}

void VoxelShape::sCollideVoxelVsConvex(const JPH::Shape *inShape1, const JPH::Shape *inShape2, JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2, JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2, const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2, const JPH::CollideShapeSettings &inCollideShapeSettings, JPH::CollideShapeCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter) {
	const VoxelShape *voxelShape = static_cast<const VoxelShape *>(inShape1);
	const JPH::ConvexShape *convexShape = static_cast<const JPH::ConvexShape *>(inShape2);

	// 1. Get the overlapping AABB in World Space
	JPH::AABox worldBounds1 = voxelShape->GetWorldSpaceBounds(inCenterOfMassTransform1, inScale1);
	JPH::AABox worldBounds2 = convexShape->GetWorldSpaceBounds(inCenterOfMassTransform2, inScale2);
	JPH::AABox intersection = worldBounds1.Intersect(worldBounds2);

	if (!intersection.IsValid()) {
		return;
	}

	// 2. Transform the intersection box into the Voxel's Local Space
	JPH::Mat44 invVoxel = inCenterOfMassTransform1.Inversed();
	JPH::AABox localIntersect = intersection.Transformed(invVoxel);

	// 3. Find the range of voxels to check
	JPH::Vec3 start = voxelShape->GetVoxelCoord(localIntersect.mMin);
	JPH::Vec3 end = voxelShape->GetVoxelCoord(localIntersect.mMax);

	// 4. Prepare transformation to move Voxel points into the Box's local space
	// worldPos = VoxelTransform * localVoxelPos
	// localBoxPos = BoxTransform.Inversed() * worldPos
	JPH::Mat44 voxelToConvex = inCenterOfMassTransform2.Inversed() * inCenterOfMassTransform1;

	for (uint32_t z = (uint32_t)start.GetZ(); z <= (uint32_t)end.GetZ(); ++z) {
		for (uint32_t y = (uint32_t)start.GetY(); y <= (uint32_t)end.GetY(); ++y) {
			for (uint32_t x = (uint32_t)start.GetX(); x <= (uint32_t)end.GetX(); ++x) {
				uint8_t type = voxelShape->mClassificationData[voxelShape->GetIndex(x, y, z)];

				// We check for solid voxels (Corner=1, Edge=2, etc.)
				if (type > 0) {
					JPH::Vec3 localVoxelPos = voxelShape->GetLocalPos(x, y, z);
					JPH::Vec3 posInConvexSpace = voxelToConvex * localVoxelPos;


					// 5. Check if the point is inside the convex shape
					JPH::AllHitCollisionCollector<JPH::CollidePointCollector> collector;
					convexShape->CollidePoint(posInConvexSpace, JPH::SubShapeIDCreator(), collector, JPH::ShapeFilter());

					// 5. Use Jolt's built-in Point-in-Convex test
					// This works for Boxes, Spheres, Capsules, etc.
					if (collector.HadHit()) {
						JPH::Vec3 worldPos = inCenterOfMassTransform1 * localVoxelPos;

						JPH::CollideShapeResult result;
						result.mContactPointOn1 = worldPos;
						result.mContactPointOn2 = worldPos;

						// Calculate a basic normal: From the center of the convex object to the voxel point
						JPH::Vec3 normal = (worldPos - inCenterOfMassTransform2.GetTranslation()).Normalized();
						result.mPenetrationAxis = -normal; // Penetration axis points from 2 to 1

						// Depth calculation (crude placeholder)
						result.mPenetrationDepth = 0.05f;

						result.mSubShapeID1 = inSubShapeIDCreator1.GetID();
						result.mSubShapeID2 = inSubShapeIDCreator2.GetID();

						ioCollector.AddHit(result);
					}
				}
			}
		}
	}
}

#ifdef JPH_DEBUG_RENDERER
void VoxelShape::Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const {
	// NOTE: To prevent lag, we only draw if the object isn't massive
	if (mClassificationData && mResolution.GetX() <= 64) {
		// Example: Draw a small dot for every 'Corner' voxel
		for (uint32_t y = 0; y < (uint32_t)mResolution.GetY(); ++y) {
			for (uint32_t z = 0; z < (uint32_t)mResolution.GetZ(); ++z) {
				for (uint32_t x = 0; x < (uint32_t)mResolution.GetX(); ++x) {
					uint8_t type = mClassificationData[GetIndex(x, y, z)];
					if (type == 1) { // TYPE_CORNER
						JPH::Vec3 local_pos = GetLocalPos(x, y, z);
						JPH::RVec3 world_pos = inCenterOfMassTransform * local_pos;

						// Draw a tiny sphere at each corner
						inRenderer->DrawMarker(world_pos, JPH::Color::sGreen, 0.05f);
					}
				}
			}
		}
	}
}
#endif

void VoxelShape::sRegister() {
	JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL);
	f.mColor = JPH::Color::sOrange;

	print_line("VOXEL REGISTRATION INITIALIZED");

	// 1. VOXEL vs VOXEL (The one you're missing)
	JPH::CollisionDispatch::sRegisterCollideShape(
			JoltCustomShapeSubType::VOXEL,
			JoltCustomShapeSubType::VOXEL,
			VoxelShape::sCollideVoxelVsVoxel);

	// 2. VOXEL vs BOX
	JPH::CollisionDispatch::sRegisterCollideShape(
			JoltCustomShapeSubType::VOXEL,
			JPH::EShapeSubType::Box,
			VoxelShape::sCollideVoxelVsConvex);

	// 3. BOX vs VOXEL (Required for symmetry)
	JPH::CollisionDispatch::sRegisterCollideShape(
			JPH::EShapeSubType::Box,
			JoltCustomShapeSubType::VOXEL,
			JPH::CollisionDispatch::sReversedCollideShape);

#if 0 // Can also be done like this but ignore for now. 
	for (EShapeSubType s1 : sAllSubShapeTypes) {
		for (EShapeSubType s2 : sAllSubShapeTypes) {
			CollisionDispatch::sRegisterCollideShape(s1, s2, sCollideConvexVsConvex);
			CollisionDispatch::sRegisterCastShape(s1, s2, sCastConvexVsConvex);
		}
	}
#endif
}
