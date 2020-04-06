#ifndef __Exporter_h__
#define __Exporter_h__

#include <vector>
#include <array>

#include "SPlisHSPlasH/Common.h"
#include "SPlisHSPlasH/Utilities/SceneLoader.h"
#include "SPlisHSPlasH/FluidModel.h"
#include "SPlisHSPlasH/TimeManager.h"

// This class handles exporting to external formats.
class Exporter {

public:

	// Flags
	static int PARTIO_EXPORT; static bool enablePartioExport;
	static int VTK_EXPORT; static bool enableVTKExport;
	static int RB_EXPORT; static bool enableRigidBodyExport;
	static int RB_VTK_EXPORT; static bool enableRigidBodyVTKExport;

	// Configs
	static int DATA_EXPORT_FPS; static Real framesPerSecond;
	static int PARTICLE_EXPORT_ATTRIBUTES; static std::string particleAttributes;
	static AlignedBox3r gridExportRegion;
	static Vector3u gridExportResolution;

private:

	// - New feature: regular grid export.
	// - Export velocity field to regular grid (inviwo volume)
	//   - Seed new particles at grid cell centers
	//   - Use current SPH method to get the velocity from the pressure solve
	
	static std::vector<Vector3r> linspace3D(AlignedBox3r region, Vector3u resolution, Vector3r* outStep = 0, Real margin = 0) {
		std::vector<Vector3r> P;
		P.resize(resolution.x * resolution.y * resolution.z);
		Vector3r step = Vector3r(region.max - region.min).array() / Vector3r(resolution).array();
		if (outStep) *outStep = step;
		for (unsigned int x = 0; x < resolution.x; x++) {
			for (unsigned int y = 0; y < resolution.y; y++) {
				for (unsigned int z = 0; z < resolution.z; z++) {
					unsigned int row = z + y * resolution.z + x * resolution.y * resolution.z;
					P[row].x = region.min.x - margin + x * step.x;
					P[row].y = region.min.y - margin + y * step.y;
					P[row].z = region.min.z - margin + z * step.z;
				}
			}
		}
		return P;
	}
	
	static float* reorderLinspace3D(Real* data, Vector3u res) {
		// z major to x major order
		size_t size = res.x * res.y * res.z;
		float* xdata = new float[size];
		for (unsigned int z = 0; z < res.z; z++) {
			for (unsigned int y = 0; y < res.y; y++) {
				for (unsigned int x = 0; x < res.x; x++) {
					unsigned int i_old = z + y * res.z + x * res.y * res.z;
					unsigned int i_new = x + y * res.x + z * res.x * res.y;
					xdata[i_new] = data[i_old];
				}
			}
		}
		return xdata;
	}

	template<typename T>
	static void writeInviwoVolume(std::string name, const void* data, float min, float max, Vector3u res, Vector3r step, bool zMajor = true) {

		// Write inviwo file format

		std::ofstream datFile(name + ".dat");
		datFile << "Rawfile: " + name + ".raw" << std::endl;

		if (zMajor) // need to swizzle because of voxel order
			datFile << "Resolution: " << res.z << " " << res.y << " " << res.x << std::endl;
		else
			datFile << "Resolution: " << res.x << " " << res.y << " " << res.z << std::endl;

#ifdef USE_DOUBLE
		datFile << "Format: Vec3FLOAT64" << std::endl;
#else
		datFile << "Format: Vec3FLOAT32" << std::endl;
#endif
		datFile << "DataRange: " << min << " " << max << std::endl;

		// set basis so that it transforms texture coords to real units
		if (zMajor) {
			datFile << "BasisVector1: " << (double)res.z * step.z << " 0 0" << std::endl;
			datFile << "BasisVector2: 0 " << (double)res.y * step.y << " 0" << std::endl;
			datFile << "BasisVector3: 0 0 " << (double)res.x * step.x << std::endl;
		}
		else {
			datFile << "BasisVector1: " << (double)res.x * step.x << " 0 0" << std::endl;
			datFile << "BasisVector2: 0 " << (double)res.y * step.y << " 0" << std::endl;
			datFile << "BasisVector3: 0 0 " << (double)res.z * step.z << std::endl;
		}
		//datFile << "Offset: 0 0 0" << std::endl; // automatically chosen by inviwo

		std::ofstream rawFile(name + ".raw", std::ios::binary);
		rawFile.write(reinterpret_cast<const char*>(data), res.x * res.y * res.z * sizeof(T));
	}

	bool m_isFirstFrameVTK;

public:

	std::string m_outputPath;

	Exporter();
	~Exporter();

	void particleExport(std::string exportName = "", std::string temporalIdentifier = "", std::string folder = "",
		bool partio = enablePartioExport, bool vtk = enableVTKExport, bool inviwo = false);
	void writeParticlesPartio(const std::string& fileName, SPH::FluidModel* model);
	void writeParticlesVTK(const std::string& fileName, SPH::FluidModel* model);

	void rigidBodyExport(std::string scenePath, std::string temporalIdentifier, bool isFirstFrame, Utilities::SceneLoader::Scene);
	void writeRigidBodiesBIN(const std::string& exportPath, std::string scene_path, std::string temporalIdentifier, bool isFirstFrame, Utilities::SceneLoader::Scene);
	void writeRigidBodiesVTK(const std::string& exportPath, std::string temporalIdentifier);

	bool isTrackingParticles() {
		return enablePartioExport || enableVTKExport;
	}

	bool isTrackingRigidBodies() {
		return enableRigidBodyExport || enableRigidBodyVTKExport;
	}

	void saveParticleSnapshot() {
		particleExport("", std::to_string(SPH::TimeManager::getCurrent()->getTime()), "snapshots", true, false, true);
	}

	void reset() {
		m_isFirstFrameVTK = true;
	}

	// VTK expects big endian
	template<typename T>
	inline void swapByteOrder(T* v)
	{
		constexpr size_t n = sizeof(T);
		uint8_t* bytes = reinterpret_cast<uint8_t*>(v);
		for (unsigned int c = 0u; c < n / 2; c++)
			std::swap(bytes[c], bytes[n - c - 1]);
	}
};

#endif
