#pragma once

#ifndef Section2D_H
#define Section2D_H

#include "stdafx.h"

using namespace std;
using namespace arma;

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
class Section2D
{
	private:
		/*
											Positive
								   /|\
									|  ----------------------->
									|/
								---------
								   /|
								  /	|
								 /	|
			ULT.		YIELD	/	|YIELD					 ULT.
	   /______|_________|______/____|__|______________________|______\  CURVATURE
	   \	  |			|	  /		|  |					  |      /
							 /		|
						    /		|
						   /		|
						  /			|
						 /			|	
						/			|
		     <----------			|
									|
								   \|/
		Negative
		*/

		// Section ID.
		int MyID;
		
		// Geometry.
		double MyGrossArea;
		double MyGrossMomentOfInertia;
		double MyMassPerUnitLength;
		double MyMassDensity;

		// Positive M-PHI Diagram.
		double MyPlasticMomentPositive;
		double MyYieldCurvaturePositive;
		double MyUltimateCurvaturePositive;
		double MyElasticModulusPositive;

		// Neutral M-PHI Diagram at Zero Curvature.
		double MyNeutralMoment;

		// Negative M-PHI Diagram.
		double MyPlasticMomentNegative;
		double MyYieldCurvatureNegative;
		double MyUltimateCurvatureNegative;
		double MyElasticModulusNegative;

		// Initialization function for Bi-Linear Model of the frame.
		void Initialize(int ID, 
					    double GrossArea,
						double GrossMomentOfInertia,
						double NeutralMoment,
						double PlasticMomentPositive, 
						double YieldCurvaturePositive, 
						double UltimateCurvaturePositive, 
						double PlasticMomentNegative, 
						double YieldCurvatureNegative, 
						double UltimateCurvatureNegative,
						double Density);

		void Clone(const Section2D &Section);
	public:
		Section2D(int ID,
				  double GrossArea,
				  double GrossMomentOfInertia,
				  double PlasticMomentPositive, 
				  double YieldCurvaturePositive, 
				  double UltimateCurvaturePositive, 
				  double PlasticMomentNegative, 
				  double YieldCurvatureNegative, 
				  double UltimateCurvatureNegative,
				  double Density);

		Section2D(int ID,
				  double GrossArea,
				  double GrossMomentOfInertia,
				  double GrossElasticModulus,
				  vector <double> Curvature,
				  double Density);

		Section2D(int ID,
				  double GrossArea,
				  double GrossMomentOfInertia,
				  double InitialMoment,
				  vector <vector<double>> PositiveCoordinates,
				  vector <vector<double>> NegativeCoordinates, 
				  double Density);

		Section2D(void);

		~Section2D(void);

		// Copy Constructor
		Section2D& operator= (const Section2D &Section);

		Section2D(const Section2D& Section);

		virtual int GetID();

		virtual double GetArea();

		virtual double GetMomentOfInertia();

		virtual double GetElasticModulus();

		virtual double GetElasticModulus(double Curvature);

		virtual double GetDensity();

		virtual double GetStress(double Strain);

		virtual double GetStrain(double Stress);

		virtual double GetPositivePlasticMoment();

		virtual double GetNegativePlasticMoment();

		virtual vector<vector<double>> GetMomentCurvatureVector();

		virtual double GetMassPerUnitLength();

		virtual void Print();
};

#endif
