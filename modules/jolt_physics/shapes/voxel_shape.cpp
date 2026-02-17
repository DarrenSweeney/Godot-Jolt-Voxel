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

void VoxelShape::sCollideVoxelVsVoxel(const JPH::Shape *inShape1, const JPH::Shape *inShape2, JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
										JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2,
										const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
										const JPH::CollideShapeSettings &inCollideShapeSettings, JPH::CollideShapeCollector &ioCollector, 
										const JPH::ShapeFilter &inShapeFilter)
{
	auto unwrap = [](const JPH::Shape *in) -> const VoxelShape * {
		const JPH::Shape *current = in;

		// Peel back every layer of Decoration (Rotated, Scaled, etc.)
		while (current->GetType() == JPH::EShapeType::Decorated) {
			current = static_cast<const JPH::DecoratedShape *>(current)->GetInnerShape();
		}

		// Now current is the actual leaf shape. Check if it's our Voxel.
		if (current->GetSubType() == JoltCustomShapeSubType::VOXEL) {
			return static_cast<const VoxelShape *>(current);
		}

		return nullptr;
	};

	const VoxelShape *shape1 = unwrap(inShape1);
	const VoxelShape *shape2 = unwrap(inShape2);

	if (!shape1 || !shape2)
		return;

	// Get the overlapping AABB in World Space
	JPH::AABox worldBounds1 = shape1->GetWorldSpaceBounds(inCenterOfMassTransform1, inScale1);
	JPH::AABox worldBounds2 = shape2->GetWorldSpaceBounds(inCenterOfMassTransform2, inScale2);
	JPH::AABox intersection = worldBounds1.Intersect(worldBounds2);

	if (!intersection.IsValid())
		return;

	// Transform intersection to Local Space of Shape 1
	JPH::Mat44 inv1 = inCenterOfMassTransform1.Inversed();
	JPH::AABox localIntersect = intersection.Transformed(inv1);

	// Loop through voxels in Shape 1
	JPH::Vec3 start = shape1->GetVoxelCoord(localIntersect.mMin);
	JPH::Vec3 end = shape1->GetVoxelCoord(localIntersect.mMax);

	for (uint32_t z = (uint32_t)start.GetZ(); z <= (uint32_t)end.GetZ(); ++z)
	{
		for (uint32_t y = (uint32_t)start.GetY(); y <= (uint32_t)end.GetY(); ++y)
		{
			for (uint32_t x = (uint32_t)start.GetX(); x <= (uint32_t)end.GetX(); ++x)
			{
				uint8_t type = shape1->mClassificationData[shape1->GetIndex(x, y, z)];
				if (type == 1 || type == 2)
				{
					JPH::Vec3 localPos = shape1->GetLocalPos(x, y, z);
					JPH::Vec3 worldPos = inCenterOfMassTransform1 * localPos;

					// Transform point into Shape 2's local space
					JPH::Vec3 localTo2 = inCenterOfMassTransform2.Inversed() * worldPos;

					if (shape2->IsSolidAt(localTo2))
					{
						JPH::CollideShapeResult result;
						result.mContactPointOn1 = worldPos;
						result.mContactPointOn2 = worldPos;
						// IMPORTANT: Normal should point from 2 to 1
						result.mPenetrationAxis = (worldPos - inCenterOfMassTransform2.GetTranslation()).Normalized();
						result.mPenetrationDepth = 0.1f;
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
