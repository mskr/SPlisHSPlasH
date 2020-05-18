#ifndef __BoundarySimulator_h__
#define __BoundarySimulator_h__

#include "SPlisHSPlasH/Common.h"
#include "Utilities/MotionTable.h"

namespace SPH
{
	class BoundarySimulator 
	{
	public:
		BoundarySimulator() {}
		virtual ~BoundarySimulator() {}
		virtual void init() {}
		virtual void timeStep() {}
		virtual void initBoundaryData() {}
		virtual void reset() {}

		void updateBoundaryForces();

		void updateBoundaryMotion();

		MotionTable loadMotionFile(std::string path);
	};
}
 
#endif
