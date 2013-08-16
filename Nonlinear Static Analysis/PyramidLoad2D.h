#pragma once

#ifndef PyramidLoad2D_H
#define PyramidLoad2D_H

#include "BeamLoad2d.h"
#include "TriangularLoad2D.h"

class PyramidLoad2D : public BeamLoad2D
{
	protected:
		//	|						  -------|
		//	|				  --------		 |------|
		//	|			 -----				 |		|
		//	|			|					 |		|
		//	|__________\|/__________________\|/____\|/_______
		//	|
		//	|
		
		// Superposition of the two load case.
		TriangularLoad2D MyLeftTriangularLoad;
		
		TriangularLoad2D MyRightTriangularLoad;

		// Magnitude for the Peak Load.
		double MyTransversePeakMagnitude;
		
		// Length till the Matnitude.
		double MyTransversePeakLength;

		// Magnitude for the Left Side of the Load.
		double MyTransverseLeftMagnitude;
		
		// Length till the loading begins.
		double MyTransverseLeftLength;
		
		// Magnitude for the Right Side of the Load.
		double MyTransverseRightMagnitude;
		
		// Length to the locationw where the loading ends.
		double MyTransverseRightLength;
		
		// Set the Triangular Loads.
		void SetTriangularLoads(void);

	public:
		void Initialize(double TransversePeakMagnitude = 0.0, 
						double TransverseLeftMagnitude = 0.0,
						double TransverseRightMagnitude = 0.0,
						double TransversePeakLength = 0.0, 
						double TransverseLeftLength = 0.0,
						double TransverseRightLength = 0.0);

		void Clone(const PyramidLoad2D &Load);

		PyramidLoad2D(Beam2D &Beam, 
					  double PeakMagnitude, 
					  double LeftMagnitude, 
					  double RightMagnitude,
					  double PeakLength, 
					  bool AddPointerToBeam = true);
		
		PyramidLoad2D(Beam2D &Beam, 
					  double PeakMagnitude, 
					  double LeftMagnitude, 
					  double RightMagnitude, 
					  double PeakLength, 
					  double LeftLength, 
					  double RightLength, 
					  bool AddPointerToBeam = true);

		PyramidLoad2D(void);

		~PyramidLoad2D(void);

		mat GetLocalLoadVector();

		PyramidLoad2D& operator= (const PyramidLoad2D &Load);
		PyramidLoad2D& operator* (double Factor);
};

#endif
