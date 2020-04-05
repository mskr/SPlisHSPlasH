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

private:

	// - New feature: regular grid export.

	struct Particle {
		double x, y, z;
		double vx, vy, vz;
	};

	// Regular grid with explicit point storage.
	struct LinSpace3D {
		using u32 = std::uint32_t;
		std::vector<Particle> P;
		std::array<u32, 3> res;
		std::array<double, 3> step;
		void generateVolumePoints(u32* volDim, double* origin, double margin, double* step) {
			P.resize(volDim[0] * volDim[1] * volDim[2]);
			for (u32 x = 0; x < volDim[0]; x++) {
				for (u32 y = 0; y < volDim[1]; y++) {
					for (u32 z = 0; z < volDim[2]; z++) {
						u32 row = z + y * volDim[2] + x * volDim[1] * volDim[2];
						P[row].x = origin[0] - margin + x * step[0];
						P[row].y = origin[1] - margin + y * step[1];
						P[row].z = origin[2] - margin + z * step[2];
					}
				}
			}
		}

		float* reorderVolumePoints(float* data, u32* res) {
			// z major to x major order
			u32 size = res[0] * res[1] * res[2];
			float* xdata = new float[size];
			for (u32 z = 0; z < res[2]; z++) {
				for (u32 y = 0; y < res[1]; y++) {
					for (u32 x = 0; x < res[0]; x++) {
						u32 i_old = z + y * res[2] + x * res[1] * res[2];
						u32 i_new = x + y * res[0] + z * res[0] * res[1];
						xdata[i_new] = data[i_old];
					}
				}
			}
			return xdata;
		}
	};

	bool m_isFirstFrameVTK;

public:

	std::string m_outputPath;

	Exporter();
	~Exporter();

	void particleExport(std::string exportName = "", std::string temporalIdentifier = "", std::string folder = "", bool partio = enablePartioExport, bool vtk = enableVTKExport);
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
		particleExport("", std::to_string(SPH::TimeManager::getCurrent()->getTime()), "snapshots", true, true);
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
