#pragma once

#ifndef PointLoad2D_H
#define PointLoad2D_H

#include "BeamLoad2d.h"

using namespace std;
using namespace arma;

/// <summary></summary>
class PointLoad2D :	public BeamLoad2D
{
	protected:
		//	|								   |
		//	|		   ||					   |
		//	|		   \/					   |
		//	|__________________________________|
		//	|			|					   |
		//	|	  L1	|          L2		   |
		//	|								   |
		
		double MyTransverseMagnitude;
		double MyTransverseLengthLeft;

		//	|								   |
		//  |				/__				   |
		//	|				\				   |
		//	|__________________________________|
		//	|				|				   |
		//	|	  L1		|          L2	   |
		//	|								   |
		double MyParallelMagnitude;
		double MyParallelLengthLeft;
	public:
		void Initialize(double TransverseMagnitude = 0.0, 
						double TransverseLengthLeft = 0.0, 
						double ParallelMagnitude = 0.0,
						double ParallelLengthLeft = 0.0);

		void Clone(const PointLoad2D &Load);

		/// Constructor: Center of the Beam.
		PointLoad2D(Beam2D &Beam, 
					double TransverseMagnitude, 
					bool AddPointerToBeam = true);

		/// <summary>Constructor: Vertical Point Load.</summary>
		PointLoad2D(Beam2D &Beam, 
					double TransverseMagnitude, 
					double TransverseLengthLeft, 
					bool AddPointerToBeam = true);

		/// <summary>Constructor: Vertical and Horizontal Point Load.</summary>
		PointLoad2D(Beam2D &Beam, 
					double TransverseMagnitude, 
					double TransverseLengthLeft, 
					double ParallelMagnitude,
					double ParallelLengthLeft, 
					bool AddPointerToBeam = true);

		PointLoad2D(Beam2D &Beam,
				    double Magnitude,
					double Length,
					char Axis,
					bool AddPointerToBeam = true);

		/// <summary>Constructor: Default.</summary>
		PointLoad2D(void);

		/// <summary>Destructor.</summary>
		~PointLoad2D(void);
		
		/// <summary></summary>
		PointLoad2D& operator= (const PointLoad2D &Load);
		PointLoad2D& operator* (double Factor);

		/// <summary>Return the Point Load Vector.</summary>
		mat GetLocalLoadVector();
};

#endif
