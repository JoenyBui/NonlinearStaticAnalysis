#pragma once

#ifndef ElasticFrame2D_H
#define ElasticFrame2D_H

#include "stdafx.h"
#include "Node2D.h"
#include "Beam2D.h"

#include "Section2D.h"

#include "BeamLoad2D.h"
#include "MomentLoad2D.h"
#include "NodeLoad2D.h"
#include "PointLoad2D.h"
#include "PyramidLoad2D.h"
#include "TriangularLoad2D.h"
#include "UniformLoad2D.h"

#include "DegreeOfFreedom2D.h"

using namespace std;
using namespace arma;

// Frame2D is the collection of the Nodes, Beams, Material Properties, and Geometric Properties.

/// <summary>
///
/// </summary>
class ElasticFrame2D
{
	private:

	protected:
		// Custom Name for the Frame.
		string MyName;

		// ID of the Frame.
		int MyID;

		// Vectors of Nodes and Beams.
		vector <Node2D> MyVectorNodes;
		vector <Beam2D> MyVectorBeams;

		vector <Section2D> MyVectorSections;

		// Loading
		vector <NodeLoad2D> MyVectorNodeLoads;

		vector <MomentLoad2D> MyVectorMomentLoads;
		vector <PointLoad2D> MyVectorPointLoads;
		vector <PyramidLoad2D> MyVectorPyramidLoads;
		vector <TriangularLoad2D> MyVectorTriangularLoads;
		vector <UniformLoad2D> MyVectorUniformLoads;

		vector <BeamLoad2D*> MyVectorBeamLoads;

		vector <DegreeOfFreedom2D> MyVectorDOF;

		// ==========================================
		//					RESULTS
		// ==========================================
		
		// Lock to check if an analysis has already ran.
		bool MyLock;

		// The Local Beam End Forces.
		vector <mat> MyLocalBeamEndForces;

		// Store the Stiffness Matrices.
		mat MyElasticStiffnessMatrix;
		mat MyGeometricStiffnessMatrix;
		mat MyTangentStiffnessMatrix;
		mat MyPermutationMatrix;
		
		// Store the Force Vectors.
		mat MyMemberForceVector;
		mat MyJointForceVector;
		mat MyCombineForceVector;

		mat MyMassVector;

		// Store the Number of DOF.
		size_t MyFreeDOF;
		size_t MyFixedDOF;
		size_t MyTotalDOF;

		// Store Displacement from the Results.
		mat MyNodesDisplacements;
		mat MyExternalForceVector;

		// =========================================
		//            FUNCTIONS
		// =========================================
		void Initialize(string Name,
						int ID);

		// Public Functions
		void Clone(const ElasticFrame2D &Frame);

		// Run the Static Analysis
		vector <mat> RunAnalysis();

	public:
		bool IncludeGeometricNonLinearity;
		bool IncludeMaterialNonLinearity;

		static const int TranslationX = 1;
		static const int TranslationY = 2;
		static const int RotationZ = 3;

		static const int DOF = 3;		

		// Constructors
		ElasticFrame2D(string Name);

		ElasticFrame2D(void);

		// Set the Equal Operator.
		ElasticFrame2D& operator= (const ElasticFrame2D &Frame);
		void operator+= (vector <Node2D> &Nodes);

		// Copy Constructor.
		ElasticFrame2D(const ElasticFrame2D& Frame);

		// Destructors
		~ElasticFrame2D(void);
		
		ElasticFrame2D GetElasticFrameGeometry();
		
		// Return Data Type Functions.
		vector <NodeLoad2D> GetNodeLoads();
		vector <MomentLoad2D> GetMomentLoads();
		vector <PointLoad2D> GetPointLoads();
		vector <PyramidLoad2D> GetPyramidLoads();
		vector <TriangularLoad2D> GetTriangularLoads();
		vector <UniformLoad2D> GetUniformLoads();

		// ===============  Add Node  ===============
		void AddNode(Node2D Node);
		void AddNode(vector <Node2D> Nodes);

		// ===============  Add Section =============
		void AddSection(Section2D Section);
		void AddSection(vector <Section2D> Sections);

		// ===============  Add Beam  ===============
		void AddBeam(Beam2D Beam);
		void AddBeam(vector <Beam2D> Beams);
		void AddBeam(Beam2D Beam, int NodeAID, int NodeBID, int MaterialID);

		// ===============  Add Node Loads  ==================
		void AddNodeLoad(NodeLoad2D NewLoad);
		void AddNodeLoad(vector <NodeLoad2D> NewLoad);
		void AddNodeLoad(NodeLoad2D Load, int NodeID);

		// ===============  Add Beam Loads  ===================
		void AddBeamLoad(BeamLoad2D &Load);

		void AddBeamLoad(MomentLoad2D Load);
		void AddBeamLoad(MomentLoad2D Load, int BeamID);
		void AddBeamLoad(vector <MomentLoad2D> Load);

		void AddBeamLoad(PointLoad2D Load);
		void AddBeamLoad(PointLoad2D Load, int BeamID);
		void AddBeamLoad(vector <PointLoad2D> Load);

		void AddBeamLoad(PyramidLoad2D Load);
		void AddBeamLoad(PyramidLoad2D Load, int BeamID);
		void AddBeamLoad(vector <PyramidLoad2D> Load);

		void AddBeamLoad(TriangularLoad2D Load);
		void AddBeamLoad(TriangularLoad2D Load, int BeamID);
		void AddBeamLoad(vector <TriangularLoad2D> Load);

		void AddBeamLoad(UniformLoad2D Load);
		void AddBeamLoad(UniformLoad2D Load, int BeamID);
		void AddBeamLoad(vector <UniformLoad2D> Load);
		
		void ConnectComponents();

		void ClearLoads();

		Node2D &GetNodeAddress(int ID);
		Beam2D &GetBeamAddress(int ID);

		Node2D GetNode(int ID);
		Beam2D GetBeam(int ID);

		vector <Node2D> GetNodes();
		vector <Beam2D> GetBeams();

		// Set the ID of the frame.
		void SetID(int ID);

		//int *GetNodalList();
		int GetIndexFromList(vector<int> List, int Item);
		int GetNumberOfFreeDOF(vector<bool> List);
		size_t GetNumberOfDOF();
	
		int GetNodeIndex(int NodeID);
		int GetBeamIndex(int BeamID);
		int GetSectionIndex(int SectionID);

		vector <int> GetNodeList();
		vector <int> GetNodeIndexesList();
		vector <bool> GetNodeFixityList();

		vector <int> GetNodeFixityIndexes();
		vector <int> GetReleaseIndexes();
		
		// Permutation Matrix
		virtual mat GetPermutationMatrix();
		
		// Stiffness Matrix
		virtual mat GetGlobalStiffnessMatrix();
		virtual mat GetGlobalGeometricStiffnessMatrix(vector <mat> LocalBeamEndForces);
		virtual mat GetPermutatedGlobalStiffnessMatrix();
		virtual mat GetFreeGlobalStiffnessMatrix();

		// Force Vector
		virtual mat GetGlobalJointForceVector();
		virtual mat GetGlobalMemberForceVector();
		virtual mat GetFreeForceVector();
	
		// Global Dispalcement Vector.
		virtual mat GetGlobalDisplacement();
		
		// Run Static Analysis without Geometric Stiffness Matrix.
		virtual vector <mat> RunStaticAnalysis(bool CalculateDisplacement = true);
		
		// Run Static Analysis with Geometric Beam End Forces
		virtual vector <mat> RunStaticAnalysis(vector <mat> LocalBeamEndForces);

		virtual vector <mat> RunStaticAnalysisOnResidualForce(vector <mat> LocalBeamEndForces, 
															  mat ResidualForce);

		
		virtual mat GetDisplacementVector(const ElasticFrame2D &Frame);

		void SetDOFIndex();

		void Print();

		size_t GetDofIndex();

		// Post-Processing Results
		// *** EXTERNAL FORCES ***
		virtual vector <mat> GetExternalBeamEndForces();
		virtual mat GetExternalBeamLocalForces(int BeamID);

		// *** INTERNAL FORCES ***
		virtual mat GetInternalForces(int BeamID);
		virtual vector <mat> GetInternalBeamEndForces();
		virtual mat GetInternalBeamLocalForces(int BeamID);


		virtual mat GetBeamGlobalDisplacement(int BeamIndex);

		virtual size_t GetNumberOfBeams();
		virtual size_t GetNumberOfNodes();

		virtual void SetBeamReleases(int BeamID, 
									 bool ReleaseNodeA, 
									 bool ReleaseNodeB);

		// Protected Functions
		virtual void CheckMomentCapacity(vector <mat> IncrementForces,
										 vector <mat> DeltaForce,
										 double Tolerance,
										 double &ScaleFactor,
										 int &HingingIndex);

		virtual void CheckForHinges(vector <mat> TotalForce, 
								    double Tolerance = 0.01);

		virtual vector <Node2D> GetDeformNodes();
		virtual vector <Node2D> GetNodeDeformation();

		virtual void SetDeformNodes(vector <Node2D> Nodes);
		virtual void SetDeformNodes(mat Displacements);
		virtual void SetNodesDisplacements(mat Displacements);

		virtual Node2D GetNodeDisplacement(int ID);

		static const int HingingIndexNotExceeded = 0;
		static const int HingingIndexOverExceeded = 1;
		static const int HingingIndexConverged = 2;

		vector <double> GetDisplacementVector(int DOF);
		vector <double> GetBeamLocalLoadVector(int DOF);
		vector <double> GetBeamGlobalLoadVector(int DOF);
		vector <double> GetShapeVector(int DOF);
		vector <double> GetMassVector(int DOF);
		vector <double> GetNodeLoadVector(int DOF);

		virtual mat GetGlobalMassVector();

		void SetName(string Name, int ID);

		bool ShowRatioTables;
		bool ShowNodes;
		bool ShowBeams;
		bool ShowNodesDisplacement;
		bool ShowNodeLoads;
		bool ShowBeamLoads;

		mat GetGlobalExternalForceVector();

		ElasticFrame2D GetPlasticFrameGeometry();
		ElasticFrame2D GetPlasticDeformFrameGeometry();
		
		mat GetGlobalInternalForceVector();
		void ModifyVectorForHinges(mat HingingMatrix);

		virtual vector <mat> GetNodesGlobalDisplacement();
		virtual mat GetNodeGlobalDisplacement(int ID);
		virtual vector <mat> GetBeamsGlobalDisplacement();
		virtual mat GetBeamLocalDisplacement(int ID);

		vector <int> GetListOfDOFIndexes(int DOF);
};

#endif