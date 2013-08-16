#pragma once

#ifndef MomentLoad2D_H
#define MomentLoad2D_H

#include "BeamLoad2D.h"

class MomentLoad2D : public BeamLoad2D
{
	protected:
		/*
				|		   (@)						|
				|___________________________________|
				|			|						|
				|	  L1	|          L2			|
		*/

		double MyMomentMagnitude;
		double MyMomentLengthLeft;

	public:
		void Initialize(double MomentMagnitude,
						double MomentLengthLeft);

		void Clone(const MomentLoad2D &Load);

		// Constructor:
		MomentLoad2D(void);

		MomentLoad2D(Beam2D &Beam,
					 double MomentMagnitude,
					 double MomentLengthLeft, 
					 bool AddPointerToBeam = true);

		~MomentLoad2D(void);

		MomentLoad2D& operator= (const MomentLoad2D &Load);
		MomentLoad2D& operator* (double Factor);

		// Return the Moment Load Vector.
		mat GetLocalLoadVector();
};

#endif
