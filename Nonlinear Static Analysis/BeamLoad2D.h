#include "stdafx.h"
#include "Node2D.h"
#include "Beam2D.h"
#include "NodeLoad2D.h"

#pragma once

using namespace std;
using namespace arma;

#ifndef BeamLoad2D_H
#define BeamLoad2D_H

// Forward Declaration.
class Beam2D;

class BeamLoad2D
{
	private:

	protected:
		Beam2D* MyBeam;

		const static int DOF = 3;
	public:
		/// <summary></summary>
		BeamLoad2D(void);

		/// <summary></summary>
		~BeamLoad2D(void);
		
		/// <summary>Assignment Operator.</summary>
		BeamLoad2D& operator= (const BeamLoad2D &Load);
		
		virtual double GetProjectedAngleofForceToBeam(char Axis);

		virtual double GetProjectedAngleofForceToBeam(double ForceAngle);

		virtual void SetBeam(Beam2D* PtrBeam);

		virtual void SetLoadPointerToBeam(bool AddPointerToBeam = true);

		// Return the Beam ID.
		int GetBeamID();

		virtual mat GetReleaseForceVector(const mat &ForceVector, int NumOfRelease);

		/// <summary>Must Override this Function.</summary>
		virtual mat GetLocalLoadVector() = 0;

		/// <summary></summary>
		virtual mat GetGlobalLoadVector();
		
		/// <summary></summary>
		virtual vector <NodeLoad2D*> GetNodalLoad();

		/// <summary></summary>
		virtual void Print(bool ShowMatrix = false);
};

#endif
