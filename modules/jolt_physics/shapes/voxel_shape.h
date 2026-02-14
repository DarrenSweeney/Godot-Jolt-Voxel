#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
#include <Jolt/Physics/Collision/Shape/GetTrianglesContext.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Renderer/DebugRenderer.h>

class VoxelShapeSettings final : public JPH::ShapeSettings {
public:
	JPH::Vec3 half_extents;
	JPH::Vec3 resolution;

	virtual JPH::ShapeSettings::ShapeResult Create() const override;
};

class VoxelShape : public JPH::Shape {
public:
	JPH::Vec3 mHalfExtents;
	JPH::Vec3 mResolution;
	const uint8_t *mDataPtr = nullptr;

	VoxelShape(const VoxelShapeSettings &inSettings, JPH::ShapeSettings::ShapeResult &outResult) :
			JPH::Shape(JPH::EShapeType::User1, JPH::EShapeSubType::User1, inSettings, outResult), // Lets take User1, there are 4.
			mHalfExtents(inSettings.half_extents),
			mResolution(inSettings.resolution) {
		if (outResult.HasError()) {
			return;
		}
		outResult.Set(this);
	}

	// --- Must Implement: Basic Geometry ---
	virtual JPH::AABox GetLocalBounds() const override { return JPH::AABox(-mHalfExtents, mHalfExtents); }
	virtual float GetInnerRadius() const override { return 0.0f; }
	virtual JPH::MassProperties GetMassProperties() const override {
		JPH::MassProperties p;
		p.mMass = 1.0f;
		p.mInertia = JPH::Mat44::sIdentity();
		return p;
	}
	virtual const JPH::PhysicsMaterial *GetMaterial(const JPH::SubShapeID &inSubShapeID) const override { return JPH::PhysicsMaterial::sDefault; }
	virtual JPH::Vec3 GetSurfaceNormal(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inLocalSurfacePosition) const override { return JPH::Vec3::sAxisY(); }

	// --- Must Implement: Collision Queries ---
	virtual void CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CollidePointCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter) const override {
		// Your logic here...
	}

	virtual void CollideSoftBodyVertices(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::CollideSoftBodyVertexIterator &inVertices, JPH::uint inNumVertices, int inCollidingShapeIndex) const override {}

	virtual bool CastRay(const JPH::RayCast &inRay, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::RayCastResult &ioHit) const override { return false; }

	virtual void CastRay(const JPH::RayCast &inRay, const JPH::RayCastSettings &inRayCastSettings, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CastRayCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter = {}) const override {}

	// --- Must Implement: Triangles (For complex collisions) ---
	virtual void GetTrianglesStart(GetTrianglesContext &ioContext, const JPH::AABox &inBox, JPH::Vec3Arg inPositionCOM, JPH::QuatArg inRotation, JPH::Vec3Arg inScale) const override {}
	virtual int GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, JPH::Float3 *outTriangleVertices, const JPH::PhysicsMaterial **outMaterials = nullptr) const override { return 0; }

	// --- Must Implement: Buoyancy ---
	virtual void GetSubmergedVolume(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, JPH::Vec3 &outCenterOfBuoyancy JPH_IF_DEBUG_RENDERER(, JPH::RVec3Arg inBaseOffset)) const override {
		outTotalVolume = GetVolume();
		outSubmergedVolume = 0.0f;
	}

	virtual JPH::Shape::Stats GetStats() const override { return JPH::Shape::Stats(sizeof(*this), 0); }
	virtual float GetVolume() const override { return mHalfExtents.GetX() * mHalfExtents.GetY() * mHalfExtents.GetZ() * 8.0f; }
	virtual JPH::uint GetSubShapeIDBitsRecursive() const override { return 0; }

#ifdef JPH_DEBUG_RENDERER
	virtual void Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override {
		inRenderer->DrawBox(inCenterOfMassTransform, GetLocalBounds(), inColor);
	}
#endif
};
