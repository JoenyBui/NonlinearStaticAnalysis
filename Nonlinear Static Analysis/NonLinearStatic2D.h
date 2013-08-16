#pragma once

#ifndef NonLinearStatic2D_H
#define NonLinearStatic2D_H

#include "stdafx.h"
#include "ElasticFrame2D.h"
#include "NonLinearFrame2D.h"

class NonLinearStatic2D
{
	private:
		// Set the Initialize.
		void Initialize();

		// Clone the Analysis.
		void Clone(const NonLinearStatic2D &Event);

	protected:
		// ======================================================================
		//						   -- PROTECTED --
		//						  SETTING THE FRAME
		// ======================================================================
#pragma region "SETTING THE FRAME"

		// *** VARIABLES ***

		// The Undeform, Unloaded Base Frame Geometry set at the beginning.
		ElasticFrame2D MyBaseFrame;

		double MyGravity;

		// An array of the Plastic Frame Analysis.
		vector <ElasticFrame2D> MyNonLinearFramePositive;
		vector <ElasticFrame2D> MyNonLinearFrameNegative;

		// A vector that describes the Static Load.  Loads are applied at time step 0.
		vector <NodeLoad2D> MyStaticNodeLoads;
		vector <NodeLoad2D> MyIncrementNodeLoads;

		// Storing the Static Beam Loading.
		vector <MomentLoad2D> MyStaticMomentLoads;
		vector <PointLoad2D> MyStaticPointLoads;
		vector <PyramidLoad2D> MyStaticPyramidLoads;
		vector <TriangularLoad2D> MyStaticTriangularLoads;
		vector <UniformLoad2D> MyStaticUniformLoads;

		// Storing the Increment Loading.
		vector <MomentLoad2D> MyIncrementMomentLoads;
		vector <PointLoad2D> MyIncrementPointLoads;
		vector <PyramidLoad2D> MyIncrementPyramidLoads;
		vector <TriangularLoad2D> MyIncrementTriangularLoads;
		vector <UniformLoad2D> MyIncrementUniformLoads;

		// Initialize the Beam Force Vector;
		void InitializeBeamForceVector();

		// Set the IncrementNodalLoad.
		void SetIncrementLoad(ElasticFrame2D &Frame, double LoadFactor);

		void SetIncrementNodeLoads(ElasticFrame2D &Frame, double LoadFactor);
		void SetIncrementMomentLoad(ElasticFrame2D &Frame, double LoadFactor);
		void SetIncrementPointLoad(ElasticFrame2D &Frame, double LoadFactor);
		void SetIncrementPyramidLoad(ElasticFrame2D &Frame, double LoadFactor);
		void SetIncrementTriangularLoad(ElasticFrame2D &Frame, double LoadFactor);
		void SetIncrementUniformLoad(ElasticFrame2D &Frame, double LoadFactor);
#pragma endregion

		// ======================================================================
		//						   -- PROTECTED --
		//						 INCREMENTAL ANALYSIS
		// ======================================================================
#pragma region "INCREMENTAL ANALYSIS"
		// *** VARIABLES ***
		vector <double> MyLoadIncrement;
		
		// Initialize the Vector Beam Local Forces
		vector <mat> MyVectorTotalDisplacementPositive;
		vector <mat> MyVectorTotalExternalForcesPositive;
		vector <mat> MyVectorTotalInternalForcesPositive;
		vector <mat> MyTotalExternalForcesAtDOFPositive;
		vector <mat> MyTotalInternalForcesAtDOFPositive;
		vector <bool> MyHingingHistoryPositive;
		vector <double> MyFinalLoadRatiosPositive;

		vector <mat> MyVectorTotalDisplacementNegative;
		vector <mat> MyVectorTotalExternalForcesNegative;
		vector <mat> MyVectorTotalInternalForcesNegative;
		vector <mat> MyTotalExternalForcesAtDOFNegative;
		vector <mat> MyTotalInternalForcesAtDOFNegative;
		vector <bool> MyHingingHistoryNegative;
		vector <double> MyFinalLoadRatiosNegative;

		// *** FUNCTIONS ***
		void AddToExternalBeamEndForces(vector <mat> BeamEndForces);
		void AddToInternalBeamEndForces(vector <mat> BeamEndForces);

		void RunStaticAnalysis();

		double GetLoadingDirectionFactor(bool IsPositiveLoading);

		vector <ElasticFrame2D> RunEulerMethod(int Increment, 
											   double Tolerance, 
											   int ExitCounterMax, 
											   bool IsPositiveLoading,
											   vector <mat> &VectorTotalDisplacement, 
											   vector <mat> &VectorTotalExternalForces,
											   vector <mat> &VectorTotalInternalForces);

		vector <ElasticFrame2D> RunIncrementalIterativeMethods(int NumOfIncrement, 
															   double NormTolerance, 
															   int ExitCounterMax, 
															   int TypeOfNonLinear, 
															   double InitialIncrementLoadRatio, 
															   bool IsPositiveLoading, 
															   vector <mat> &VectorTotalDisplacement, 
															   vector <mat> &VectorTotalExternalForces,
															   vector <mat> &VectorTotalInternalForces,
															   vector <mat> &TotalExternalForcesAtDOF,
															   vector <mat> &TotalInternalForcesAtDOF,
															   vector <double> &FinalLoadRatios, 
															   vector <bool> &HingingHistory);

		
		double GetLoadRatioNewtonRhapsonMethod();
		double GetLoadRatioConstantArcLengthMethod(double InitialLoadRatio, 
												   mat DisplacementInitial,
												   mat DisplacementReferenceLoad, 
												   mat DisplacementResidualLoad);

		double GetNormModifiedAbsolute(mat IterationDisplacement, mat TotalDisplacement);
		double GetNormModifiedEuclidean(mat IterationDisplacement, mat TotalDisplacement);
		double GetNormMaximum(mat IterationDisplacement, mat TotalDisplacement);

		double GetAutomaticLoadIncrementation(double PreviousLoadRatio, 
											  double PreviousIteration, 
											  double DesiredIteration, 
											  double ExponentParameter, 
											  double MaxLoadRatio);

		mat GetHingingPermutationMatrix(ElasticFrame2D &InitialFrame, 
										ElasticFrame2D &FinalFrame);
#pragma endregion

		// ======================================================================
		//						   -- PROTECTED --
		//						 RESISTANCE FUNCTION
		// ======================================================================
#pragma region "RESISTANCE FUNCTION"

		// *** VARIABLES ***
		// *** POSITIVE ****
		vector <vector<double>> MyLocationVectorPositiveTranslationX;
		vector <vector<double>> MyLocationVectorPositiveTranslationY;
		vector <vector<double>> MyLocationVectorPositiveRotationZ;

		vector <vector<double>> MyDisplacementVectorPositiveTranslationX;
		vector <vector<double>> MyDisplacementVectorPositiveTranslationY;
		vector <vector<double>> MyDisplacementVectorPositiveRotationZ;

		vector <vector<double>> MyShapeVectorPositiveTranslationX;
		vector <vector<double>> MyShapeVectorPositiveTranslationY;
		vector <vector<double>> MyShapeVectorPositiveRotationZ;

		vector <vector<double>> MyMassVectorPositiveTranslationX;
		vector <vector<double>> MyMassVectorPositiveTranslationY;
		vector <vector<double>> MyMassVectorPositiveRotationZ;

		vector <vector<double>> MyBeamLoadVectorPositiveTranslationX;
		vector <vector<double>> MyBeamLoadVectorPositiveTranslationY;
		vector <vector<double>> MyBeamLoadVectorPositiveRotationZ;

		vector <vector<double>> MyNodeLoadVectorPositiveTranslationX;
		vector <vector<double>> MyNodeLoadVectorPositiveTranslationY;
		vector <vector<double>> MyNodeLoadVectorPositiveRotationZ;

		// *** NEGATIVE ***
		vector <vector<double>> MyLocationVectorNegativeTranslationX;
		vector <vector<double>> MyLocationVectorNegativeTranslationY;
		vector <vector<double>> MyLocationVectorNegativeRotationZ;

		vector <vector<double>> MyDisplacementVectorNegativeTranslationX;
		vector <vector<double>> MyDisplacementVectorNegativeTranslationY;
		vector <vector<double>> MyDisplacementVectorNegativeRotationZ;

		vector <vector<double>> MyShapeVectorNegativeTranslationX;
		vector <vector<double>> MyShapeVectorNegativeTranslationY;
		vector <vector<double>> MyShapeVectorNegativeRotationZ;

		vector <vector<double>> MyMassVectorNegativeTranslationX;
		vector <vector<double>> MyMassVectorNegativeTranslationY;
		vector <vector<double>> MyMassVectorNegativeRotationZ;

		vector <vector<double>> MyBeamLoadVectorNegativeTranslationX;
		vector <vector<double>> MyBeamLoadVectorNegativeTranslationY;
		vector <vector<double>> MyBeamLoadVectorNegativeRotationZ;

		vector <vector<double>> MyNodeLoadVectorNegativeTranslationX;
		vector <vector<double>> MyNodeLoadVectorNegativeTranslationY;
		vector <vector<double>> MyNodeLoadVectorNegativeRotationZ;

		// *** FUNCTIONS ***
		// Setting Vector.
		void SetDisplacementVector();
		void SetMassVector();
		void SetNodeLoadVector();
		void SetBeamLoadVector();
		void SetShapeVector();

		vector <double> GetNormalizeDisplacementVector(vector <double> DisplacementVector);
#pragma endregion

	public:
		bool ShowInternalEndForce;
		bool ShowExternalEndForce;

		// Default Constructor.
		NonLinearStatic2D(void);

		// Destructor.
		~NonLinearStatic2D(void);

		// Copy Constructor
		NonLinearStatic2D(const NonLinearStatic2D &Event);

		// Assignment Operator
		NonLinearStatic2D& operator= (const NonLinearStatic2D &Event);

		// ======================================================================
		//						   -- PUBLIC --
		//						  SETTING THE FRAME
		// ======================================================================
#pragma region "SETTING THE FRAME"

		// Set the Frame Geometry.
		void SetFrameGeometry(ElasticFrame2D Frame);

		ElasticFrame2D GetFrameGeometry();

		// Add the Static Load at '0' time.
		void AddStaticLoad(NodeLoad2D Load);
		void AddStaticLoad(MomentLoad2D Load);
		void AddStaticLoad(PointLoad2D Load);
		void AddStaticLoad(PyramidLoad2D Load);
		void AddStaticLoad(TriangularLoad2D Load);
		void AddStaticLoad(UniformLoad2D Load);

		void AddStaticLoad(vector <NodeLoad2D> Load);
		void AddStaticLoad(vector <MomentLoad2D> Load);
		void AddStaticLoad(vector <PointLoad2D> Load);
		void AddStaticLoad(vector <PyramidLoad2D> Load);
		void AddStaticLoad(vector <TriangularLoad2D> Load);
		void AddStaticLoad(vector <UniformLoad2D> Load);

		vector <NodeLoad2D> GetStaticNodeLoad2D();
		vector <MomentLoad2D> GetStaticMomentLoad2D();
		vector <PointLoad2D> GetStaticPointLoad2D();
		vector <PyramidLoad2D> GetStaticPyramidLoad2D();
		vector <TriangularLoad2D> GetStaticTriangularLoad2D();
		vector <UniformLoad2D> GetStaticUniformLoad2D();

		// Increment the Load.
		void AddIncrementalLoad(NodeLoad2D Load);
		void AddIncrementalLoad(MomentLoad2D Load);
		void AddIncrementalLoad(PointLoad2D Load);
		void AddIncrementalLoad(PyramidLoad2D Load);
		void AddIncrementalLoad(TriangularLoad2D Load);
		void AddIncrementalLoad(UniformLoad2D Load);

		void AddIncrementalLoad(vector <NodeLoad2D> Load);
		void AddIncrementalLoad(vector <MomentLoad2D> Load);
		void AddIncrementalLoad(vector <PointLoad2D> Load);
		void AddIncrementalLoad(vector <PyramidLoad2D> Load);
		void AddIncrementalLoad(vector <TriangularLoad2D> Load);
		void AddIncrementalLoad(vector <UniformLoad2D> Load);

		vector <NodeLoad2D> GetIncrementalNodeLoad2D();
		vector <MomentLoad2D> GetIncrementalMomentLoad2D();
		vector <PointLoad2D> GetIncrementalPointLoad2D();
		vector <PyramidLoad2D> GetIncrementalPyramidLoad2D();
		vector <TriangularLoad2D> GetIncrementalTriangularLoad2D();
		vector <UniformLoad2D> GetIncrementalUniformLoad2D();
#pragma endregion

		// ======================================================================
		//						   -- PUBLIC --
		//						 INCREMENTAL ANALYSIS
		// ======================================================================
#pragma region "INCREMENTAL ANALYSIS"
		static const int Euler = 0;
		static const int NewtonRhapson = 1;
		static const int ArcLength = 2;

		void RunAnalysis(int TypeOfNonLinear, 
						 int NumOfIncrement = 10, 
						 double NormTolerance = 0.01,
						 int ExitCounterMax = 20,
						 double InitialIncrementLoadRatio = 0.10);

		vector <Node2D> GetNodeLoadHistory(int ID, bool IsPositive);
#pragma endregion

		// ======================================================================
		//						   -- PUBLIC --
		//						 RESISTANCE FUNCTION
		// ======================================================================
#pragma region "RESISTANCE FUNCTION"

		// Shape Vector.
		static const int TranslationX = 1;
		static const int TranslationY = 2;
		static const int RotationZ = 3;

		// Return the Shape Vector.
		vector <double> GetShapeVector(int DOF, 
									   int LoadStep,
									   bool IsLoadingPositive);
		vector <double> GetMassVector(int DOF, 
									  int LoadStep,
									  bool IsLoadingPositive);
		vector <double> GetBeamLoadVector(int DOF, 
										  int LoadStep,
										  bool IsLoadingPositive);
		vector <double> GetNodeLoadVector(int DOF, 
										  int LoadStep,
										  bool IsLoadingPositive);

		// Vector of the Load and Mass Factor
		vector <double> GetMassFactorVector(int DOF,
											bool IsLoadingPositive );
		vector <double> GetLoadFactorVector(int DOF,
											bool IsLoadingPositive);
		
		vector <double> GetTotalHistoryLoad(int DOF,
											bool IsLoadingPositive);

		double GetTotalLoad(int LoadStep, 
			                int DOF,
							bool IsLoadingPositive);

		double GetTotalReferenceLoad(int DOF);

		void ConnectAllStaticAnalysis();

		double GetTotalMass(int DOF, 
							int LoadStep, 
							bool IsLoadingPositive);

		int GetNodeIDWithLargestDeformation(int NodeDOF,
											int LoadIncrement,
											bool IsPositiveLoading);

		vector <vector<double>> GetResistanceFunction(int NodeID, 
													  int NodeDOF, 
													  int LoadDOF, 
													  bool IsPositiveLoading);

		vector <vector<double>> GetLoadMassFactor(int NodeID,
												  int NodeDOF,
												  int LoadDOF,
												  bool IsPositiveLoading);

#pragma endregion

		void SetGravity(double Gravity);
		
		double GetGravity();
};

#endif