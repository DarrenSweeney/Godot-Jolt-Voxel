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
}

bool VoxelShape::IsSolidAt(JPH::Vec3Arg inLocalPoint) const
{
	if (!GetLocalBounds().Contains(inLocalPoint) || !mClassificationData)
		return false;

	JPH::Vec3 coord = GetVoxelCoord(inLocalPoint);
	int index = GetIndex((uint32_t)coord.GetX(), (uint32_t)coord.GetY(), (uint32_t)coord.GetZ());

	return mClassificationData[index] > 0;
}

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
		if (current->GetSubType() == JoltCustomShapeSubType::VOXEL) {
			return static_cast<const VoxelShape *>(current);
		}
		return nullptr;
	};

	const VoxelShape *shape1 = unwrap(inShape1);
	const VoxelShape *shape2 = unwrap(inShape2);

	if (!shape1 || !shape2)
		return;

	// AABB Intersection in World Space to limit the search area
	JPH::AABox worldBounds1 = shape1->GetWorldSpaceBounds(inCenterOfMassTransform1, inScale1);
	JPH::AABox worldBounds2 = shape2->GetWorldSpaceBounds(inCenterOfMassTransform2, inScale2);
	JPH::AABox intersection = worldBounds1.Intersect(worldBounds2);

	if (!intersection.IsValid())
		return;

	// Define a lambda for the point-vs-volume pass to avoid code duplication
	auto processPass = [&](const VoxelShape *pShape, const VoxelShape *vShape,
							   JPH::Mat44Arg pTransform, JPH::Mat44Arg vTransform,
							   const JPH::SubShapeIDCreator &pID, const JPH::SubShapeIDCreator &vID,
							   bool isSwapped) {

		JPH::Mat44 invP = pTransform.Inversed();
		JPH::Mat44 invV = vTransform.Inversed();
		JPH::AABox localIntersect = intersection.Transformed(invP);

		JPH::Vec3 start = pShape->GetVoxelCoord(localIntersect.mMin);
		JPH::Vec3 end = pShape->GetVoxelCoord(localIntersect.mMax);

		for (uint32_t z = (uint32_t)start.GetZ(); z <= (uint32_t)end.GetZ(); ++z)
		{
			for (uint32_t y = (uint32_t)start.GetY(); y <= (uint32_t)end.GetY(); ++y)
			{
				for (uint32_t x = (uint32_t)start.GetX(); x <= (uint32_t)end.GetX(); ++x)
				{
					uint8_t type = pShape->mClassificationData[pShape->GetIndex(x, y, z)];

					// Only check Corners (1) and Edges (2)
					if (type == 1 || type == 2)
					{
						JPH::Vec3 pLocalPos = pShape->GetLocalPos(x, y, z);
						JPH::Vec3 worldPos = pTransform * pLocalPos;
						JPH::Vec3 vLocalPos = invV * worldPos;

						if (vShape->IsSolidAt(vLocalPos))
						{
							JPH::CollideShapeResult result;
							result.mContactPointOn1 = worldPos;
							result.mContactPointOn2 = worldPos;

							// Calculate Normal from the volume shape (vShape)
							JPH::Vec3 localNormal = vShape->GetSurfaceNormalAt(vLocalPos);
							JPH::Vec3 worldNormal = vTransform.Multiply3x3(localNormal);

							// Jolt Rule: Penetration axis must point from Shape 2 to Shape 1
							result.mPenetrationAxis = isSwapped ? worldNormal : -worldNormal;
							result.mPenetrationDepth = 0.1f; // Adjust based on voxel size
							result.mSubShapeID1 = isSwapped ? vID.GetID() : pID.GetID();
							result.mSubShapeID2 = isSwapped ? pID.GetID() : vID.GetID();

							ioCollector.AddHit(result);
						}
					}
				}
			}
		}
	};

	// Pass 1: Shape 1 (Points) vs Shape 2 (Volume)
	processPass(shape1, shape2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, false);

	// Pass 2: Shape 2 (Points) vs Shape 1 (Volume)
	processPass(shape2, shape1, inCenterOfMassTransform2, inCenterOfMassTransform1, inSubShapeIDCreator2, inSubShapeIDCreator1, true);
}

#ifdef JPH_DEBUG_RENDERER
void VoxelShape::Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const {
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
}
#endif

void VoxelShape::sRegister()
{
	JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL);
	f.mConstruct = []() -> Shape * { return new VoxelShape; };
	f.mColor = JPH::Color::sOrange;

	JPH::CollisionDispatch::sRegisterCollideShape(JoltCustomShapeSubType::VOXEL, JoltCustomShapeSubType::VOXEL, sCollideVoxelVsVoxel);
}
