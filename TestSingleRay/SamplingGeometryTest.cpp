#include "SamplingUtils.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace {
	using namespace osc;

	bool approx(float a, float b, float eps = 1e-5f)
	{
		return std::fabs(a - b) <= eps;
	}

	void testLegacyDirections()
	{
		DirectionSamplingConfig cfg;
		cfg.mode = DirectionSamplingMode::Legacy66Sphere;
		const auto dirs = generateDirectionSamples(cfg);
		assert(dirs.size() == 66);

		bool hasNorthPole = false;
		bool hasSouthPole = false;
		for (const auto& dir : dirs) {
			assert(approx(length(dir), 1.0f, 1e-4f));
			hasNorthPole = hasNorthPole || (approx(dir.x, 0.0f) && approx(dir.y, 0.0f) && approx(dir.z, 1.0f));
			hasSouthPole = hasSouthPole || (approx(dir.x, 0.0f) && approx(dir.y, 0.0f) && approx(dir.z, -1.0f));
		}
		assert(hasNorthPole);
		assert(hasSouthPole);
	}

	void testGoldenHemisphereDirections()
	{
		DirectionSamplingConfig cfg;
		cfg.mode = DirectionSamplingMode::GoldenHemisphere;
		cfg.goldenPointCount = 200;
		cfg.hemisphereMinZ = -0.05f;

		const auto dirs = generateDirectionSamples(cfg);
		assert(dirs.size() == 105);

		bool hasNorthPole = false;
		for (const auto& dir : dirs) {
			assert(approx(length(dir), 1.0f, 1e-4f));
			assert(dir.z > cfg.hemisphereMinZ - 1e-6f);
			hasNorthPole = hasNorthPole || (approx(dir.x, 0.0f) && approx(dir.y, 0.0f) && approx(dir.z, 1.0f));
		}
		assert(hasNorthPole);
	}

	void testCarriageSamples()
	{
		CarriageSamplingConfig cfg;
		const auto points = generateCarriageSamples(cfg);
		assert(points.size() == static_cast<size_t>(cfg.sampleCount));

		for (const auto& point : points) {
			const float radius = std::sqrt(point.x * point.x + point.y * point.y);
			assert(approx(point.z, cfg.z, 1e-5f));
			assert(radius >= cfg.innerRadius - 1e-5f);
			assert(radius <= cfg.outerRadius + 1e-5f);
		}
	}

	void testBottomZFilter()
	{
		const std::vector<vec3f> points = {
			vec3f(0.f, 0.f, -2.f),
			vec3f(1.f, 0.f, -2.f),
			vec3f(0.f, 1.f, -1.f),
		};
		const std::vector<int> indices = { 10, 11, 12 };

		std::vector<vec3f> filteredPoints;
		std::vector<int> filteredIndices;
		filterBottomZPoints(points, indices, -2.f, true, filteredPoints, filteredIndices);
		assert(filteredPoints.size() == 1);
		assert(filteredIndices.size() == 1);
		assert(filteredIndices[0] == 12);
		assert(approx(filteredPoints[0].z, -1.f));

		filterBottomZPoints(points, indices, -2.f, false, filteredPoints, filteredIndices);
		assert(filteredPoints.size() == points.size());
		assert(filteredIndices.size() == indices.size());
	}
}

int main()
{
	testLegacyDirections();
	testGoldenHemisphereDirections();
	testCarriageSamples();
	testBottomZFilter();
	std::cout << "SamplingGeometryTest passed" << std::endl;
	return 0;
}
