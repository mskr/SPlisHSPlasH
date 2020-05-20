#ifndef __PBDRigidBody_h__
#define __PBDRigidBody_h__

#include "SPlisHSPlasH/Common.h"
#include "SPlisHSPlasH/RigidBodyObject.h"
#include "Simulation/RigidBody.h"
#include "SPlisHSPlasH/TimeManager.h"

#include "Utilities/MotionTable.h"

namespace SPH 
{	
	class PBDRigidBody : public RigidBodyObject 
	{
	protected: 
		PBD::RigidBody *m_rigidBody;

		///////////////////////////////////////////////
		// ScriptedMotion: no actual dynamics, but also not static.
		// Please move to own rigid body file.
		///////////////////////////////////////////////

		bool m_isScripted;
		MotionTable m_motionTable;
		Vector3r m_initialPosition;

	public:
		PBDRigidBody(PBD::RigidBody *rigidBody) :
			m_rigidBody(rigidBody),
			m_isScripted(false),
			m_initialPosition(rigidBody->getPosition()) {}

		virtual bool isDynamic() const { return m_rigidBody->getMass() != 0.0; }

		virtual Real const getMass() const { return m_rigidBody->getMass(); }
		virtual Vector3r const& getPosition() const { return m_rigidBody->getPosition(); }
		virtual Vector3r const& getVelocity() const { return m_rigidBody->getVelocity(); }
		virtual void setVelocity(const Vector3r &v) { m_rigidBody->setVelocity(v); }
		virtual Matrix3r const& getRotation() const { return m_rigidBody->getRotationMatrix(); }
		virtual Vector3r const& getAngularVelocity() const { return m_rigidBody->getAngularVelocity(); }
		virtual void setAngularVelocity(const Vector3r &v) { m_rigidBody->setAngularVelocity(v); }

		virtual void setPosition(const Vector3r &x) 
		{ 
			m_rigidBody->setPosition(x); 
			m_rigidBody->getGeometry().updateMeshTransformation(m_rigidBody->getPosition(), m_rigidBody->getRotationMatrix());
		}

		virtual void setRotation(const Matrix3r &r) 
		{ 
			m_rigidBody->setRotation(Quaternionr(r)); m_rigidBody->rotationUpdated(); 
			m_rigidBody->getGeometry().updateMeshTransformation(m_rigidBody->getPosition(), m_rigidBody->getRotationMatrix());
		}

		virtual void addForce(const Vector3r &f) 
		{ 
			const Real dt = SPH::TimeManager::getCurrent()->getTimeStepSize();
			m_rigidBody->getVelocity() += (1.0/ m_rigidBody->getMass()) * f * dt; 
		}

		virtual void addTorque(const Vector3r &t) 
		{ 
			const Real dt = SPH::TimeManager::getCurrent()->getTimeStepSize();
			m_rigidBody->getAngularVelocity() += m_rigidBody->getInertiaTensorInverseW() * t * dt; 
		}

		// transformation local to:
		// p_world = R R_MAT^T (R_initial p_local + x_initial - x_MAT) + x
		virtual Vector3r getWorldSpacePosition() const 
		{ 
			const Matrix3r R2 = (m_rigidBody->getRotation() * m_rigidBody->getRotationMAT().inverse()).matrix();
			const Vector3r x = R2 * m_rigidBody->getPositionInitial_MAT() + m_rigidBody->getPosition();
			return x; 
		}

		virtual Matrix3r getWorldSpaceRotation() const 
		{ 
			return (m_rigidBody->getRotation() * m_rigidBody->getRotationMAT().inverse() * m_rigidBody->getRotationInitial()).matrix();
		}

		virtual const std::vector<Vector3r> &getVertices() const { return *m_rigidBody->getGeometry().getVertexData().getVertices(); };
		virtual const std::vector<Vector3r> &getVertexNormals() const { return m_rigidBody->getGeometry().getMesh().getVertexNormals(); };
		virtual const std::vector<unsigned int> &getFaces() const { return m_rigidBody->getGeometry().getMesh().getFaces(); };

		///////////////////////////////////////////////
		// ScriptedMotion: no actual dynamics, but also not static.
		// Please move to own rigid body file.
		///////////////////////////////////////////////

		void setScriptedMotion(MotionTable& motionTable) {
			m_isScripted = true;
			m_motionTable = motionTable;
		}

		bool isScripted() const { return m_isScripted; }

		void updateScriptedMotion() {
			Real t = TimeManager::getCurrent()->getTime();
			Real h = TimeManager::getCurrent()->getTimeStepSize();
			const Vector3r newPos = {
				m_motionTable.sample(t, "x"),
				m_motionTable.sample(t, "y"),
				m_motionTable.sample(t, "z")
			};
			const auto vel = (newPos - getPosition()) / h;
			setVelocity({ vel.x(), vel.y(), vel.z() });
			setPosition(m_initialPosition + newPos);
		}
	};
}

#endif 