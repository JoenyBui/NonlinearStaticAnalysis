#pragma once

#ifndef UniformLoad2D_H
#define UniformLoad2D_H

#include "BeamLoad2D.h"

using namespace std;
using namespace arma;

/// <summary></summary>
class UniformLoad2D : public BeamLoad2D
{
	protected:
		//				 |-----------|
		//	|			\|/			\|/
		//	|______________________________________
		//	|			--------------/
		//
		//

		// Transverse Magintude Uniform Distribution
		double MyTransverseMagnitude;

		// Transverse Length from the Left of the Beam before the Distribution Starts.
		double MyTransverseLeftLength;

		// Transverse Length from the Right of the Beam where the distribution ends.
		double MyTransverseRightLength;

		// Parallel Magnitude Uniform Distribution
		double MyParallelMagnitude;
		
		// Parallel Length from the Left of the Beam before distribution begin.
		double MyParallelLeftLength;
		
		// Parallel Length from the Right of the Beam when distribution ends.
		double MyParallelRightLength;

	public:
		void Initialize(double TransverseMagnitude = 0.0,
			            double TransverseLeftLength = 0.0, 
						double TransverseRightLength = 0.0,
						double ParallelMagnitude = 0.0,
						double ParallelLeftLength = 0.0, 
						double ParallelRightLength = 0.0);

		void Clone(const UniformLoad2D &Load);

		/// <summary> A Unifrom Distribution across the entire beam: only for transverse.</summary>
		UniformLoad2D(Beam2D &Beam, 
					  double TransverseMagnitude, 
					  bool AddPointerToBeam = true);
		
		/// <summary> A Uniform distribution transverse and parallel . </summary>
		UniformLoad2D(Beam2D &Beam, 
					  double TransverseMagnitude, 
					  double TransverseLeftLength, 
					  double TransverseRightLength,
					  double ParallelMagnitude, 
					  double ParallelLeftLength, 
					  double ParallelRightLength, 
					  bool AddPointerToBeam = true);
		
		UniformLoad2D(Beam2D &Beam,
			          double UniformMagnitude,
					  double LeftLength,
					  double RightLength,
					  char Axis,
					  bool AddPointerToBeam = true);

		/// <summary></summary>
		UniformLoad2D(void);

		/// <summary></summary>
		~UniformLoad2D(void);
		
		/// <summary>Assignment Operator</summary>
		UniformLoad2D& operator= (const UniformLoad2D &Load);
		UniformLoad2D& operator* (double Factor);

		/// <summary></summary>
		mat GetLocalLoadVector();
		
		double GetLeftFixedEndParallelForce(double Length);
		double GetLeftFixedEndTransverseForce(double Length);
		double GetLeftFixedEndMoment(double Length);
		double GetRightFixedEndParallelForce(double Length);
		double GetRightFixedEndTransverseForce(double Length);
		double GetRightFixedEndMoment(double Length);
};

#endif
