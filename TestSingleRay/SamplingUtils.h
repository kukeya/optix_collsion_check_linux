#pragma once

#include "gdt/math/vec.h"

#include <string>
#include <vector>

namespace osc {
	using namespace gdt;

	enum class DirectionSamplingMode {
		Legacy66Sphere,
		GoldenHemisphere,
		AxisFive,
		AxisNine,
	};

	struct DirectionSamplingConfig {
		DirectionSamplingMode mode = DirectionSamplingMode::GoldenHemisphere;
		int goldenPointCount = 200;
		float hemisphereMinZ = -0.05f;
	};

	struct CarriageSamplingConfig {
		int sampleCount = 158;
		float innerRadius = 1.5f;
		float outerRadius = 25.0f;
		float z = -23.0f;
	};

	bool parseDirectionSamplingMode(const std::string& raw, DirectionSamplingMode& out);
	const char* directionSamplingModeName(DirectionSamplingMode mode);
	bool defaultPruneBottomZMin(DirectionSamplingMode mode);

	std::vector<vec3f> generateDirectionSamples(const DirectionSamplingConfig& config);
	std::vector<vec3f> generateCarriageSamples(const CarriageSamplingConfig& config);
	void filterBottomZPoints(const std::vector<vec3f>& points,
		const std::vector<int>& indices,
		float zMin,
		bool enabled,
		std::vector<vec3f>& filteredPoints,
		std::vector<int>& filteredIndices);
}
