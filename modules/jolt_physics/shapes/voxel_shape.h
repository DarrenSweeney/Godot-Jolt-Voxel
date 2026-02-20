#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
#include <Jolt/Physics/Collision/Shape/GetTrianglesContext.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Renderer/DebugRenderer.h>

#include "jolt_custom_shape_type.h"

class VoxelShapeSettings final : public JPH::ShapeSettings
{
public:
	JPH::Vec3 half_extents;
	JPH::Vec3 resolution;
	const uint8_t *voxel_corner_data = nullptr;
	size_t voxel_corner_data_size = 0;
	const uint8_t *voxel_edge_data = nullptr;
	size_t voxel_edge_data_size = 0;

	const uint8_t *voxel_bitfield_data = nullptr;
	size_t voxel_bitfield_data_size = 0;

	virtual JPH::ShapeSettings::ShapeResult Create() const override;
};

class VoxelShape : public JPH::Shape
{
public:
	JPH::Vec3 mHalfExtents;
	JPH::Vec3 mResolution;

	const uint8_t *mVoxelCornerData = nullptr;
	size_t mVoxelCornerDataSize = 0;

	const uint8_t *mVoxelEdgeData = nullptr;
	size_t mVoxelEdgeDataSize = 0;

	const uint8_t *mVoxelBitfieldData = nullptr;
	size_t mVoxelBitfieldSize = 0;

	VoxelShape() :
			JPH::Shape(JPH::EShapeType::User1, JoltCustomShapeSubType::VOXEL) { }

	VoxelShape(const VoxelShapeSettings &inSettings, JPH::ShapeSettings::ShapeResult &outResult) :
			JPH::Shape(JPH::EShapeType::User1, JoltCustomShapeSubType::VOXEL, inSettings, outResult),
			mHalfExtents(inSettings.half_extents),
			mResolution(inSettings.resolution),

			mVoxelCornerData(inSettings.voxel_corner_data),
			mVoxelCornerDataSize(inSettings.voxel_corner_data_size),

			mVoxelEdgeData(inSettings.voxel_edge_data),
			mVoxelEdgeDataSize(inSettings.voxel_edge_data_size),

			mVoxelBitfieldData(inSettings.voxel_bitfield_data),
			mVoxelBitfieldSize(inSettings.voxel_bitfield_data_size)

	{
		if (outResult.HasError())
			return;

		outResult.Set(this);
	}

	static void sCollideVoxelVsVoxelLocal(
			const VoxelShape *inShape1, // The shape we are testing points FROM
			const VoxelShape *inShape2, // The shape we are testing volume AGAINST
			JPH::Mat44Arg inCenterOfMassTransform1, // Transform for shape 1
			JPH::Mat44Arg inCenterOfMassTransform2, // Transform for shape 2
			const JPH::SubShapeIDCreator &inSubShapeIDCreator1,
			const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
			const JPH::AABox &inIntersection, // The pre-calculated AABB intersection
			JPH::CollideShapeCollector &ioCollector, // The Jolt collector
			bool inFlip // TRUE if this is Pass 2
	);

	static void sCollideVoxelVsVoxel(const JPH::Shape *inShape1, const JPH::Shape *inShape2, JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
								JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2,
								const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
								const JPH::CollideShapeSettings &inCollideShapeSettings, JPH::CollideShapeCollector &ioCollector,
								const JPH::ShapeFilter &inShapeFilter);

	bool IsSolidAt(JPH::Vec3Arg inLocalPoint) const;

	// Helper to convert Local Meters -> Integer Voxel Coordinates
	JPH::Vec3 GetVoxelCoord(JPH::Vec3Arg inLocalPoint) const
	{
		JPH::Vec3 unit_coord = (inLocalPoint + mHalfExtents) / (2.0f * mHalfExtents) * mResolution;
		return JPH::Vec3(
				JPH::Clamp((uint32_t)unit_coord.GetX(), 0u, (uint32_t)mResolution.GetX() - 1),
				JPH::Clamp((uint32_t)unit_coord.GetY(), 0u, (uint32_t)mResolution.GetY() - 1),
				JPH::Clamp((uint32_t)unit_coord.GetZ(), 0u, (uint32_t)mResolution.GetZ() - 1));
	}

	// Converts integer grid coordinates back to local meters (center of voxel)
	JPH::Vec3 GetLocalPos(uint32_t x, uint32_t y, uint32_t z) const
	{
		JPH::Vec3 voxel_size = (2.0f * mHalfExtents) / mResolution;
		return -mHalfExtents + (JPH::Vec3((float)x, (float)y, (float)z) * voxel_size) + (voxel_size * 0.5f);
	}

	int GetIndex(uint32_t x, uint32_t y, uint32_t z) const
	{
		return y * (mResolution.GetX() * mResolution.GetZ()) + z * mResolution.GetX() + x;
	}

	// --- Must Implement: Basic Geometry ---
	virtual JPH::AABox GetLocalBounds() const override { return JPH::AABox(-mHalfExtents, mHalfExtents); }
	virtual float GetInnerRadius() const override { return 0.0f; }
	virtual JPH::MassProperties GetMassProperties() const override
	{
		JPH::MassProperties p;
		p.mMass = 1.0f;
		p.mInertia = JPH::Mat44::sIdentity();
		return p;
	}
	virtual const JPH::PhysicsMaterial *GetMaterial(const JPH::SubShapeID &inSubShapeID) const override { return JPH::PhysicsMaterial::sDefault; }
	virtual JPH::Vec3 GetSurfaceNormal(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inLocalSurfacePosition) const override { return JPH::Vec3::sAxisY(); }

	JPH::Vec3 DecodeNormal(uint8_t mask) const;
	//JPH::Vec3 GetSurfaceNormalAt(JPH::Vec3Arg inLocalPoint) const;

	// --- Must Implement: Collision Queries ---
	virtual void CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CollidePointCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter) const override;

	// Empty implementation
	virtual void CollideSoftBodyVertices(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::CollideSoftBodyVertexIterator &inVertices, JPH::uint inNumVertices, int inCollidingShapeIndex) const override {}
	virtual bool CastRay(const JPH::RayCast &inRay, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::RayCastResult &ioHit) const override { return false; }
	virtual void CastRay(const JPH::RayCast &inRay, const JPH::RayCastSettings &inRayCastSettings, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CastRayCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter = {}) const override {}
	virtual void GetTrianglesStart(GetTrianglesContext &ioContext, const JPH::AABox &inBox, JPH::Vec3Arg inPositionCOM, JPH::QuatArg inRotation, JPH::Vec3Arg inScale) const override { }
	virtual int GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, JPH::Float3 *outTriangleVertices, const JPH::PhysicsMaterial **outMaterials = nullptr) const override { return 0; }

	// --- Must Implement: Buoyancy ---
	virtual void GetSubmergedVolume(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, JPH::Vec3 &outCenterOfBuoyancy JPH_IF_DEBUG_RENDERER(, JPH::RVec3Arg inBaseOffset)) const override
	{
		outTotalVolume = GetVolume();
		outSubmergedVolume = 0.0f;
	}

	virtual JPH::Shape::Stats GetStats() const override { return JPH::Shape::Stats(sizeof(*this), 0); }
	virtual float GetVolume() const override { return mHalfExtents.GetX() * mHalfExtents.GetY() * mHalfExtents.GetZ() * 8.0f; }
	virtual JPH::uint GetSubShapeIDBitsRecursive() const override { return 0; }

#ifdef JPH_DEBUG_RENDERER
	virtual void Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif

	static void sRegister();
};
