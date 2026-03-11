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

#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Renderer/DebugRenderer.h>
#endif

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

	const uint8_t *voxel_data = nullptr;
	size_t voxel_data_size = 0;

	virtual JPH::ShapeSettings::ShapeResult Create() const override;
};

static const JPH::Vec3 normal_lut[26] {
	// Cardinal directions
	{ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 },
	{ -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 },
	// Edge directions
	{ 1, 1, 0 }, { 1, 0, 1 }, { 0, 1, 1 },
	{ -1, 1, 0 }, { -1, 0, 1 }, { 0, -1, 1 },
	{ 1, -1, 0 }, { 1, 0, -1 }, { 0, 1, -1 },
	{ -1, -1, 0 }, { -1, 0, -1 }, { 0, -1, -1 },
	// Corner directions
	{ 1, 1, 1 }, { 1, 1, -1 }, { 1, -1, 1 }, { 1, -1, -1 },
	{ -1, 1, 1 }, { -1, 1, -1 }, { -1, -1, 1 }, { -1, -1, -1 }
};

enum VoxelType : uint8_t {
	VoxelType_Empty,
	VoxelType_Corner,
	VoxelType_Edge,
	VoxelType_Face,
	VoxelType_Inside
};

class VoxelShape : public JPH::Shape
{
public:
	float mDensity = 1000.0f;
	JPH::Vec3 mHalfExtents;
	JPH::Vec3 mResolution;

	const uint8_t *mVoxelCornerData = nullptr;
	size_t mVoxelCornerDataSize = 0;

	const uint8_t *mVoxelEdgeData = nullptr;
	size_t mVoxelEdgeDataSize = 0;

	const uint8_t *mVoxelBitfieldData = nullptr;
	size_t mVoxelBitfieldSize = 0;

	const uint8_t *mVoxelData = nullptr;
	size_t mVoxelDataSize = 0;


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
			mVoxelBitfieldSize(inSettings.voxel_bitfield_data_size),

			mVoxelData(inSettings.voxel_data),
			mVoxelDataSize(inSettings.voxel_data_size)

	{
		if (outResult.HasError())
			return;

		outResult.Set(this);
	}

	static void sCollidePointsVsGrid(
			const VoxelShape *inPointsShape,
			const VoxelShape *inGridShape,
			JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
			JPH::Mat44Arg inTransformPointsToWorld,
			JPH::Mat44Arg inTransformGridToWorld,
			const JPH::SubShapeIDCreator &inSubShapeIDCreator1,
			const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
			const JPH::CollideShapeSettings &inCollideShapeSettings,
			JPH::CollideShapeCollector &ioCollector);

	static void sCollideVoxelVsVoxelLocal(
			const VoxelShape *inShape1, // The shape we are testing points FROM
			const VoxelShape *inShape2, // The shape we are testing volume AGAINST
			JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
			JPH::Mat44Arg inCenterOfMassTransform1, // Transform for shape 1
			JPH::Mat44Arg inCenterOfMassTransform2, // Transform for shape 2
			const JPH::SubShapeIDCreator &inSubShapeIDCreator1,
			const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
			const JPH::CollideShapeSettings &inCollideShapeSettings,
			JPH::CollideShapeCollector &ioCollector // The Jolt collector
	);

	static void sCollideVoxelVsVoxel(const JPH::Shape *inShape1, const JPH::Shape *inShape2, JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2,
								JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2,
								const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2,
								const JPH::CollideShapeSettings &inCollideShapeSettings, JPH::CollideShapeCollector &ioCollector,
								const JPH::ShapeFilter &inShapeFilter);

	virtual void CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CollidePointCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter) const override;


	void GetVoxelMetadata(JPH::Vec3Arg inGridPos, uint8_t &outType, JPH::Vec3 &outNormal) const;
	bool IsSolidAt(const JPH::Vec3 &voxelGridPos) const;
	int GetIndex(uint32_t x, uint32_t y, uint32_t z) const;

	bool CheckVoxelCollision(JPH::Vec3 &voxelGridPos) const;
	JPH::Vec3 GetLocalPos(const JPH::Vec3 &argIndex) const;
	JPH::Vec3 GetGridIndex(const JPH::Vec3 &argLocalPos) const;
	JPH::Vec3 FindSurfaceVoxel(JPH::Vec3 solidVoxelPos) const;

	/// Set density of the shape (kg / m^3)
	void SetDensity(float inDensity);
	// Get density of the shape (kg / m^3)
	float GetDensity() const;
	
	void GetSupportingFace(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inDirection, JPH::Vec3Arg inScale, JPH::Mat44Arg inCenterOfMassTransform, JPH::Shape::SupportingFace &outVertices) const;
	void GetSupportingFace_(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inDirection, JPH::Vec3Arg inScale, JPH::Mat44Arg inCenterOfMassTransform, JPH::Shape::SupportingFace &outVertices, JPH::Vec3 inContactPoint) const;

	// --- Must Implement: Basic Geometry ---
	virtual JPH::AABox GetLocalBounds() const override { return JPH::AABox(-mHalfExtents, mHalfExtents); }
	virtual float GetInnerRadius() const override { return 0.0f; }
	virtual JPH::Vec3 GetSurfaceNormal(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inLocalSurfacePosition) const override;
	virtual JPH::MassProperties GetMassProperties() const override;
	virtual const JPH::PhysicsMaterial *GetMaterial(const JPH::SubShapeID &inSubShapeID) const override;

	// Empty implementation
	virtual void CollideSoftBodyVertices(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::CollideSoftBodyVertexIterator &inVertices, JPH::uint inNumVertices, int inCollidingShapeIndex) const override {}
	virtual bool CastRay(const JPH::RayCast &inRay, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::RayCastResult &ioHit) const override;
	virtual void CastRay(const JPH::RayCast &inRay, const JPH::RayCastSettings &inRayCastSettings, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CastRayCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter = {}) const override;
	virtual void GetTrianglesStart(GetTrianglesContext &ioContext, const JPH::AABox &inBox, JPH::Vec3Arg inPositionCOM, JPH::QuatArg inRotation, JPH::Vec3Arg inScale) const override { }
	virtual int GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, JPH::Float3 *outTriangleVertices, const JPH::PhysicsMaterial **outMaterials = nullptr) const override { return 0; }
	virtual JPH::Shape::Stats GetStats() const override { return JPH::Shape::Stats(sizeof(*this), 0); }
	virtual JPH::uint GetSubShapeIDBitsRecursive() const override { return 0; }

	virtual void GetSubmergedVolume(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, JPH::Vec3 &outCenterOfBuoyancy JPH_IF_DEBUG_RENDERER(, JPH::RVec3Arg inBaseOffset)) const override
	{
		outTotalVolume = GetVolume();
		outSubmergedVolume = 0.0f;
	}

	virtual float GetVolume() const override { return mHalfExtents.GetX() * mHalfExtents.GetY() * mHalfExtents.GetZ() * 8.0f; }

	static void sRegister();

#ifdef JPH_DEBUG_RENDERER
	virtual void Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override {}
#endif // JPH_DEBUG_RENDERER
};
