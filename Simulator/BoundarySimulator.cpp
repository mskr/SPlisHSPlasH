#include "BoundarySimulator.h"
#include "SPlisHSPlasH/TimeManager.h"
#include "SPlisHSPlasH/Simulation.h"
#include "SPlisHSPlasH/BoundaryModel.h"

#include <fstream>
#include <Utilities/StringTools.h>

using namespace SPH;

void BoundarySimulator::updateBoundaryForces()
{
	Real h = TimeManager::getCurrent()->getTimeStepSize();
	Simulation *sim = Simulation::getCurrent();
	const unsigned int nObjects = sim->numberOfBoundaryModels();
	for (unsigned int i = 0; i < nObjects; i++)
	{
		BoundaryModel *bm = sim->getBoundaryModel(i);
		RigidBodyObject *rbo = bm->getRigidBodyObject();
		if (rbo->isDynamic())
		{
			Vector3r force, torque;
			bm->getForceAndTorque(force, torque);
			rbo->addForce(force);
			rbo->addTorque(torque);
			bm->clearForceAndTorque();
		}
	}
}

void BoundarySimulator::updateBoundaryMotion()
{
	Real t = TimeManager::getCurrent()->getTime();
	Simulation *sim = Simulation::getCurrent();
	const unsigned int nObjects = sim->numberOfBoundaryModels();
	for (unsigned int i = 0; i < nObjects; i++)
	{
		BoundaryModel *bm = sim->getBoundaryModel(i);
		if (bm->isScripted())
		{
			RigidBodyObject *rbo = bm->getRigidBodyObject();
			const auto pos = rbo->getPosition();
			const auto motion = bm->getMotion(t);
			rbo->setPosition({ pos.x() + motion.x(), pos.y() + motion.y(), pos.z() + motion.z() });
		}
	}
}

MotionTable BoundarySimulator::loadMotionFile(std::string path) {
	std::ifstream file(path);
	using namespace Utilities;
	const auto header = StringTools::line(file);
	const auto metadata = StringTools::strings(header);
	MotionTable result;
	result.name = metadata[0];
	result.numQuantities = atoi(metadata[1].c_str());
	result.numSamples = atoi(metadata[2].c_str());
	result.periodic = atoi(metadata[3].c_str());
	const auto quantities = StringTools::strings(StringTools::line(file));
	if (quantities.size() != result.numQuantities) {
		printf("%d quantities specified, but %d found in motion file %s.", result.numQuantities, quantities.size(), result.name);
		exit(1);
	}
	int sampleCount = 0;
	for (std::string data; std::getline(file, data);) {
		const auto samples = StringTools::strings(data);
		if (samples.size() != result.numQuantities) {
			printf("%d quantities specified, but %d samples found in line %d of motion file %s.", result.numQuantities, samples.size(), sampleCount+2, result.name);
			exit(1);
		}
		for (int i = 0; i < result.numQuantities; i++) {
			result.data[quantities[i]].push_back(atof(samples[i].c_str()));
		}
		sampleCount++;
	}
	if (sampleCount != result.numSamples) {
		printf("%d sample lines specified, but %d found in motion file %s.", result.numSamples, sampleCount, result.name);
		exit(1);
	}
	return result;
}