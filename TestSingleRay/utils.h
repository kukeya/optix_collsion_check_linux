#pragma once
#include "gdt/math/vec.h"
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace gdt;


static inline void rotateToolPoint(const vec3f& toolPoint,
	const vec3f& targetDir,
	vec3f& rotated)
{
	const vec3f defaultUp = vec3f(0.f, 0.f, -1.f);

	// 计算旋转轴
	vec3f axis = cross(defaultUp, targetDir);
	float axisLength = length(axis);

	// 如果旋转轴长度接近零，说明 targetDir 与 defaultUp 平行或反平行
	if (axisLength < 1e-6f) {
		// 如果方向相同，不需要旋转
		if (dot(defaultUp, targetDir) > 0) {
			rotated = toolPoint;
		}
		else {
			// 如果方向相反，直接取反
			rotated = -toolPoint;
		}
		return;
	}

	// 归一化旋转轴
	axis = axis / axisLength;

	// 计算旋转角度
	float cosTheta = dot(defaultUp, targetDir);
	float theta = acosf(cosTheta);

	// 进行旋转变换
	rotated = toolPoint * cosf(theta) +
		cross(axis, toolPoint) * sinf(theta) +
		axis * dot(axis, toolPoint) * (1.f - cosf(theta));
}

void loadPointsOFF(std::string filename, std::vector<vec3f>& vertex) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "无法打开文件: " << filename << std::endl;
		return;
	}

	std::string header;
	file >> header;

	// 检查是否为 NOFF 或 OFF 格式
	bool hasNormals = false;
	if (header == "NOFF" || header == "noff") {
		hasNormals = true;
	}
	else if (header != "OFF" && header != "off") {
		std::cerr << "无效的 OFF 文件: " << filename << " (header: " << header << ")" << std::endl;
		file.close();
		return;
	}

	int numVertices, numFaces, numEdges;
	file >> numVertices >> numFaces >> numEdges;
	vertex.resize(numVertices);

	for (int i = 0; i < numVertices; ++i) {
		// 读取顶点坐标
		file >> vertex[i].x >> vertex[i].y >> vertex[i].z;

		// 如果是 NOFF 格式，跳过法向量 (nx, ny, nz)
		if (hasNormals) {
			float nx, ny, nz;
			file >> nx >> ny >> nz;
		}
	}

	file.close();
}

void writeOBJwithPointsAndDirections(const std::string outFilename, std::vector<vec3f>& points, const vec3f& currentDir) {

	
	
	std::ofstream outFile(outFilename, std::ios::out | std::ios::trunc);
	if (!outFile.is_open()) {
		std::cerr << "无法打开文件: " << outFilename << std::endl;
		return;
	}

	outFile << "# OBJ file\n";
	float lineLength = 1000.f;

	// 输出每个点及其延伸终点
	for (auto& pt : points) {
		outFile << "v " << pt.x << " " << pt.y << " " << pt.z << "\n";
	}
	for (auto& pt : points) {
		vec3f end = pt + currentDir * lineLength;
		outFile << "v " << end.x << " " << end.y << " " << end.z << "\n";
	}

	// 使用“l”指令定义线段 (OBJ 中顶点索引从1开始)
	if (currentDir != vec3f(0, 0, 0)) {
		int n = static_cast<int>(points.size());
		for (int i = 1; i <= n; i++) {
			outFile << "l " << i << " " << i + n << "\n";
		}
	}

	outFile.close();
	std::cout << "已输出OBJ文件: " << outFilename << std::endl;

}

void writeOBJwithPoints(const std::string outFilename, std::vector<vec3f>& points) {

	std::ofstream outFile(outFilename, std::ios::out | std::ios::trunc);
	if (!outFile.is_open()) {
		std::cerr << "无法打开文件: " << outFilename << std::endl;
		return;
	}

	outFile << "# OBJ file\n";

	// 输出每个点及其延伸终点
	for (auto& pt : points) {
		outFile << "v " << pt.x << " " << pt.y << " " << pt.z << "\n";
	}

	outFile.close();
	std::cout << "已输出OBJ文件: " << outFilename << std::endl;

}


	void outputToolModel(const vec3f& p,
		const vec3f& currentDir,
		const vec3f& toolSample,
		const std::vector<vec3f>& toolSamplePoints,
		const std::vector<vec3f>& carriage,
		const std::string outFilename) {

	vec3f rotatedToolSample;
	rotateToolPoint(toolSample, currentDir, rotatedToolSample);
	vec3f toolOrigin = p - rotatedToolSample;

	std::vector<vec3f> finalPoints;
	finalPoints.reserve(toolSamplePoints.size() + carriage.size());
	for (auto& pt : toolSamplePoints) {
		vec3f rotatedPt;
		rotateToolPoint(pt, currentDir, rotatedPt);
		finalPoints.push_back(toolOrigin + rotatedPt);
	}
		for (auto& pt : carriage)
		{
			vec3f rotatedPt;
			rotateToolPoint(pt, currentDir, rotatedPt);
			finalPoints.push_back(toolOrigin + rotatedPt);
		}

		float axisLength = 1.0f;
		for (const auto& pt : finalPoints) {
			axisLength = std::max(axisLength, length(pt - toolOrigin));
		}
		axisLength *= 1.5f;

		std::ofstream outFile(outFilename, std::ios::out | std::ios::trunc);
		if (!outFile.is_open()) {
			std::cerr << "无法打开文件: " << outFilename << std::endl;
			return;
		}

		outFile << "# Tool pose debug OBJ\n";
		for (const auto& pt : finalPoints) {
			outFile << "v " << pt.x << " " << pt.y << " " << pt.z << "\n";
		}

		const int toolOriginIdx = static_cast<int>(finalPoints.size()) + 1;
		const int contactPointIdx = toolOriginIdx + 1;
		const int spindleDirIdx = toolOriginIdx + 2;
		const int machiningDirIdx = toolOriginIdx + 3;
		const vec3f spindleEnd = toolOrigin + currentDir * axisLength;
		const vec3f machiningEnd = toolOrigin - currentDir * axisLength;

		outFile << "v " << toolOrigin.x << " " << toolOrigin.y << " " << toolOrigin.z << "\n";
		outFile << "v " << p.x << " " << p.y << " " << p.z << "\n";
		outFile << "v " << spindleEnd.x << " " << spindleEnd.y << " " << spindleEnd.z << "\n";
		outFile << "v " << machiningEnd.x << " " << machiningEnd.y << " " << machiningEnd.z << "\n";

		outFile << "l " << toolOriginIdx << " " << contactPointIdx << "\n";
		outFile << "l " << toolOriginIdx << " " << spindleDirIdx << "\n";
		outFile << "l " << toolOriginIdx << " " << machiningDirIdx << "\n";
		outFile.close();

		std::cout << "已输出OBJ文件: " << outFilename << std::endl;
	}
