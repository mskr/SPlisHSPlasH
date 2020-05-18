#ifndef __MotionTable_h__
#define __MotionTable_h__

#include "SPlisHSPlasH/Common.h"

#include <map>
#include <vector>

struct MotionTable {
	std::string name;
	int numQuantities;
	int numSamples;
	bool periodic;
	std::map<std::string, std::vector<Real>> data;

	Real sample(Real t, std::string quantity) {
		const auto timepoints = data["time"];
		Real last = 0.0;
		for (size_t i = 0; i < timepoints.size(); i++) {
			const auto timepoint = timepoints[i];
			if (t < timepoint) {
				const auto a = (t - last) / (timepoint - last);
				const auto samples = data[quantity];
				if (i == 0) return samples[0];
				return samples[i - 1] + a * (samples[i] - samples[i - 1]);
			}
			last = timepoint;
		}
	}
};

#endif