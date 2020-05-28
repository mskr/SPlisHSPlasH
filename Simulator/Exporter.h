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
	static std::string excludeModels;
	static std::string modelAttrMapping;

private:

	// - New feature: regular grid export.
	// - Export velocity field to regular grid (inviwo volume)
	//   - Seed new particles at grid cell centers
	//   - Use current SPH method to get the velocity from the pressure solve
	
	static std::vector<Vector3r> linspace3D(AlignedBox3r region, Vector3u resolution, Vector3r* outStep = 0, Real margin = 0);

	template<typename T, unsigned int components>
	static void writeInviwoVolume(std::string name, const void* data, Vector3u res, Vector3r step, Real min = -1.0, Real max = -1.0, bool zMajor = true);

	bool m_isFirstFrameVTK;

public:

	std::string m_outputPath;

	Exporter();
	~Exporter();

	void particleExport(std::string exportName = "", std::string temporalIdentifier = "", std::string folder = "",
		bool partio = enablePartioExport, bool vtk = enableVTKExport, bool ascii = false, bool inviwo = false);
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
		particleExport("", std::to_string(SPH::TimeManager::getCurrent()->getTime()), "snapshots", false, false, true, false);
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
