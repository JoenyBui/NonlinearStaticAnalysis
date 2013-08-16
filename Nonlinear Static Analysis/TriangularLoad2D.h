#pragma once

#ifndef TriangularLoad2D_H
#define TriangularLoad2D_H

#include "BeamLoad2d.h"

using namespace std;
using namespace arma;

class TriangularLoad2D : public BeamLoad2D
{
	protected:
		/*
		
			|						  -------| t2
			|				  --------		 |
			|			 -----				 |		
			|		t1	|					 |
			|__________\|/__________________\|/____________
			|
			|
		
			p1												p2
			/----------------------------------------------/
		
		*/

		// Magnitude for the Left Side of the Load.
		double MyTransverseLeftMagnitude;
		
		// Length till the loading begins.
		double MyTransverseLeftLength;
		
		// Magnitude for the Right Side of the Load.
		double MyTransverseRightMagnitude;
		
		// Length to the locationw where the loading ends.
		double MyTransverseRightLength;

		// Parallel Forces (Axial Forces)
		// Parallel Force Magnitude at the Left Edge.
		double MyParallelLeftMagnitude;
		
		// Parallel Force Magnitude at the Right Edge.
		double MyParallelRightMagnitude;

	public:
		void Initialize(double TransverseLeftMagnitude = 0.0,
						double TransverseRightMagnitude = 0.0,
						double TransverseLeftLength = 0.0,
						double TransverseRightLength = 0.0,
						double ParallelLeftMagnitude = 0.0,
						double ParallelRightMagnitude = 0.0);

		void Clone(const TriangularLoad2D &Load);

		// Constructors:
		TriangularLoad2D(Beam2D &Beam, 
						 double LeftMagnitude, 
						 double RightMagnitude, 
						 bool AddPointerToBeam = true);
		
		TriangularLoad2D(Beam2D &Beam, 
						 double LeftMagnitude, 
						 double RightMagnitude, 
						 double LeftLength, 
						 double RightLength, 
						 bool AddPointerToBeam = true);

		TriangularLoad2D(Beam2D &Beam, 
						 double TransverseLeftMagnitude, 
						 double TransverseRightMagnitude, 
						 double TransverseLeftLength, 
						 double TransverseRightLength, 
						 double ParallelLeftMagnitude,
						 double ParallelRightMagnitude, 
						 bool AddPointerToBeam = true);

		TriangularLoad2D(Beam2D &Beam,
						 double LinearLeftMagnitude,
						 double LinearRightMagnitude,
						 char Axis, 
						 bool AddPointerToBeam = true);

		TriangularLoad2D(void);

		// Destructors:
		~TriangularLoad2D(void);

		TriangularLoad2D& operator= (const TriangularLoad2D &Load);
		TriangularLoad2D& operator* (double Factor);

		mat GetLocalLoadVector();

		double GetLowerFixedEndParallelForce(double Length,
											 double LowerMagnitude, 
											 double HigherMagnitude);

		double GetLowerFixedEndTransverseForce(double Length, 
											   double LowerMagnitude, 
											   double LowerLength, 
											   double HigherMagnitude, 
											   double HigherLength);

		double GetLowerFixedEndMoment(double Length, 
									 double LowerMagnitude, 
									 double LowerLength, 
									 double HigherMagnitude, 
									 double HigherLength);

		double GetHigherFixedEndParallelForce(double Length,
											  double LowerMagnitude, 
											  double HigherMagnitude);

		double GetHigherFixedEndTransverseForce(double Length, 
											   double LowerMagnitude, 
											   double LowerLength, 
											   double HigherMagnitude, 
											   double HigherLength);

		double GetHigherFixedEndMoment(double Length, 
									   double LowerMagnitude, 
									   double LowerLength, 
									   double HigherMagnitude, 
									   double HigherLength);
};

#endif