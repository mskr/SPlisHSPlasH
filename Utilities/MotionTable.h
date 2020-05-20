#ifndef __MotionTable_h__
#define __MotionTable_h__

#include "SPlisHSPlasH/Common.h"
#include "Utilities/StringTools.h"
#include <map>
#include <vector>
#include <fstream>

struct MotionTable {
	std::string name;
	int numQuantities;
	int numSamples;
	bool periodic;
	std::map<std::string, std::vector<Real>> data;

	MotionTable() : name("empty"), numQuantities(0), numSamples(0), periodic(false) {}

	MotionTable(std::string path) {
		std::ifstream file(path);
		using namespace Utilities;
		const auto header = StringTools::line(file);
		const auto metadata = StringTools::strings(header);
		this->name = metadata[0];
		this->numQuantities = atoi(metadata[1].c_str());
		this->numSamples = atoi(metadata[2].c_str());
		this->periodic = atoi(metadata[3].c_str());
		const auto quantities = StringTools::strings(StringTools::line(file));
		if (quantities.size() != this->numQuantities) {
			printf("%d quantities specified, but %d found in motion file %s.", this->numQuantities, quantities.size(), this->name);
			exit(1);
		}
		int sampleCount = 0;
		for (std::string data; std::getline(file, data);) {
			const auto samples = StringTools::strings(data);
			if (samples.size() != this->numQuantities) {
				printf("%d quantities specified, but %d samples found in line %d of motion file %s.", this->numQuantities, samples.size(), sampleCount+2, this->name);
				exit(1);
			}
			for (int i = 0; i < this->numQuantities; i++) {
				this->data[quantities[i]].push_back(atof(samples[i].c_str()));
			}
			sampleCount++;
		}
		if (sampleCount != this->numSamples) {
			printf("%d sample lines specified, but %d found in motion file %s.", this->numSamples, sampleCount, this->name);
			exit(1);
		}
	}

	Real sample(Real t, std::string quantity) {
		const auto timepoints = data["time"];
		const auto samples = data[quantity];
		if (periodic) {
			const auto t_end = timepoints[timepoints.size() - 1];
			t = fmod(t, t_end);
		}
		Real last = 0.0;
		for (size_t i = 0; i < timepoints.size(); i++) {
			const auto timepoint = timepoints[i];
			if (t < timepoint) {
				const auto a = (t - last) / (timepoint - last);
				const auto i_l = (i == 0) ? samples.size() - 1 : i - 1;
				return samples[i_l] + a * (samples[i] - samples[i_l]);
			}
			last = timepoint;
		}
	}
};

#endif