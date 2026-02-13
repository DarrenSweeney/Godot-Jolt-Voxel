#pragma once

#if 0
#include <Jolt/Physics/Collision/Shape/Shape.h>

class VoxelShapeSettings : public JPH::ShapeSettings {
public:
	JPH::Vec3 half_extents;
	JPH::UVec3 resolution;
	JPH::Array<uint8_t> data;

	virtual ShapeResult Create() const override;
};

class VoxelShape : public JPH::Shape {
public:
	// Required metadata for Jolt
	static void sRegister();

	// The logic Jolt calls when a shape overlaps our bounding box
	virtual void CollideShape(const JPH::CollideShapeSettings &inSettings, JPH::Vec3Arg inScale, JPH::Mat44Arg inCenterOfMassTransform, const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2, JPH::CollideShapeCollector &ioCollector) const override;

	// ... other required overrides (GetLocalBounds, etc)
};

#endif
