#include "voxel_shape.h"

#include "../spaces/jolt_query_collectors.h"

#include "Jolt/Physics/Collision/CastResult.h"
#include "Jolt/Physics/Collision/RayCast.h"
#include "Jolt/Physics/Collision/TransformedShape.h"


JPH::ShapeSettings::ShapeResult VoxelShapeSettings::Create() const {
	if (mCachedResult.IsEmpty()) {
		new VoxelShape(*this, mCachedResult);
	}

	return mCachedResult;
}

