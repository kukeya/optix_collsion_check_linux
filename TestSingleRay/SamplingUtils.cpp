#include "SamplingUtils.h"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace osc {
	namespace {
		const float kPi = 3.14159265358979323846f;
		const float kGoldenAngle = 3.1415926f * (3.0f - std::sqrt(5.0f));

		static std::string toLowerAscii(std::string value)
		{
			for (size_t i = 0; i < value.size(); ++i) {
				value[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(value[i])));
			}
			return value;
		}

		static vec3f safeNormalize(const vec3f& v)
		{
			const float len = length(v);
			if (len <= 1e-8f) return v;
			return v / len;
		}

		static void appendRing(std::vector<vec3f>& directions, float z, int count, float startAngle)
		{
			if (count <= 0) return;
			const float radius = std::sqrt(std::max(0.0f, 1.0f - z * z));
			const float step = (2.0f * kPi) / static_cast<float>(count);
			for (int i = 0; i < count; ++i) {
				const float theta = startAngle + step * static_cast<float>(i);
				directions.push_back(safeNormalize(vec3f(std::cos(theta) * radius, std::sin(theta) * radius, z)));
			}
		}

		static void appendPermutationRing(std::vector<vec3f>& directions, float z)
		{
			const float a = 0.4082483f;
			const float b = 0.8164966f;
			const vec3f rawPoints[] = {
				vec3f(-a, -b, z), vec3f(a, -b, z),
				vec3f(-b, -a, z), vec3f(b, -a, z),
				vec3f(-b, a, z), vec3f(b, a, z),
				vec3f(-a, b, z), vec3f(a, b, z),
			};
			for (size_t i = 0; i < sizeof(rawPoints) / sizeof(rawPoints[0]); ++i) {
				directions.push_back(safeNormalize(rawPoints[i]));
			}
		}
	}

	bool parseDirectionSamplingMode(const std::string& raw, DirectionSamplingMode& out)
	{
		const std::string value = toLowerAscii(raw);
		if (value == "golden_hemisphere" || value == "golden" || value == "hemisphere") {
			out = DirectionSamplingMode::GoldenHemisphere;
			return true;
		}
		if (value == "legacy_66_sphere" || value == "legacy_66" || value == "sphere_66" || value == "full_sphere_66") {
			out = DirectionSamplingMode::Legacy66Sphere;
			return true;
		}
		return false;
	}

	const char* directionSamplingModeName(DirectionSamplingMode mode)
	{
		switch (mode) {
		case DirectionSamplingMode::Legacy66Sphere:
			return "legacy_66_sphere";
		case DirectionSamplingMode::GoldenHemisphere:
			return "golden_hemisphere";
		default:
			return "unknown";
		}
	}

	bool defaultPruneBottomZMin(DirectionSamplingMode mode)
	{
		return mode == DirectionSamplingMode::GoldenHemisphere;
	}

	std::vector<vec3f> generateDirectionSamples(const DirectionSamplingConfig& config)
	{
		std::vector<vec3f> directions;
		if (config.mode == DirectionSamplingMode::Legacy66Sphere) {
			directions.reserve(66);
			directions.push_back(vec3f(0.f, 0.f, -1.f));
			appendRing(directions, -0.9238795f, 4, -0.5f * kPi);
			appendRing(directions, -0.8164966f, 4, -0.75f * kPi);
			appendRing(directions, -0.7071068f, 4, -0.5f * kPi);
			appendPermutationRing(directions, -0.4082483f);
			appendRing(directions, -0.3826834f, 4, -0.5f * kPi);
			appendRing(directions, 0.0f, 16, -0.5f * kPi);
			appendRing(directions, 0.3826834f, 4, -0.5f * kPi);
			appendPermutationRing(directions, 0.4082483f);
			appendRing(directions, 0.7071068f, 4, -0.5f * kPi);
			appendRing(directions, 0.8164966f, 4, -0.75f * kPi);
			appendRing(directions, 0.9238795f, 4, -0.5f * kPi);
			directions.push_back(vec3f(0.f, 0.f, 1.f));
			return directions;
		}

		const int sampleCount = std::max(1, config.goldenPointCount);
		const float offset = 2.0f / static_cast<float>(sampleCount);
		directions.reserve(static_cast<size_t>(sampleCount) + 1);
		directions.push_back(vec3f(0.f, 0.f, 1.f));
		for (int i = 0; i < sampleCount; ++i) {
			const float y = static_cast<float>(i) * offset - 1.0f + (offset * 0.5f);
			const float radiusXZ = std::sqrt(std::max(0.0f, 1.0f - y * y));
			const float theta = kGoldenAngle * static_cast<float>(i);
			const float x = std::cos(theta) * radiusXZ;
			const float z = std::sin(theta) * radiusXZ;
			if (z > config.hemisphereMinZ) {
				directions.push_back(safeNormalize(vec3f(x, y, z)));
			}
		}
		return directions;
	}

	std::vector<vec3f> generateCarriageSamples(const CarriageSamplingConfig& config)
	{
		std::vector<vec3f> points;
		if (config.sampleCount <= 0) return points;
		if (config.outerRadius <= 0.0f || config.outerRadius < config.innerRadius) return points;

		points.reserve(static_cast<size_t>(config.sampleCount));
		const float inner2 = config.innerRadius * config.innerRadius;
		const float outer2 = config.outerRadius * config.outerRadius;
		for (int i = 0; i < config.sampleCount; ++i) {
			const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(config.sampleCount);
			const float radius = std::sqrt(inner2 + t * (outer2 - inner2));
			const float theta = kGoldenAngle * static_cast<float>(i);
			points.push_back(vec3f(std::cos(theta) * radius, std::sin(theta) * radius, config.z));
		}
		return points;
	}

	void filterBottomZPoints(const std::vector<vec3f>& points,
		const std::vector<int>& indices,
		float zMin,
		bool enabled,
		std::vector<vec3f>& filteredPoints,
		std::vector<int>& filteredIndices)
	{
		if (!enabled) {
			filteredPoints = points;
			filteredIndices = indices;
			return;
		}

		filteredPoints.clear();
		filteredIndices.clear();
		const size_t count = std::min(points.size(), indices.size());
		float coordScale = 1.0f;
		for (size_t i = 0; i < count; ++i) {
			coordScale = std::max(coordScale, std::fabs(points[i].z));
		}
		const float eps = std::max(1e-6f, coordScale * 1e-6f);

		for (size_t i = 0; i < count; ++i) {
			if (std::fabs(points[i].z - zMin) <= eps) continue;
			filteredPoints.push_back(points[i]);
			filteredIndices.push_back(indices[i]);
		}
	}
}
