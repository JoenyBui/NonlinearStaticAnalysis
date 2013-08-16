
#pragma once

#ifndef Beam2D_H
#define Beam2D_H

#include "stdafx.h"
#include "Node2D.h"
#include "BeamLoad2D.h"
#include "Section2D.h"

using namespace std;
using namespace arma;

// Forward Declaration.
class Node2D;
class BeamLoad2D;
class Section2D;

///<summary>Base Class for Beam Element.  The element assume uniform cross section across the beam.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
class Beam2D
{
	protected:
		int MyID;
			
		Section2D *MySection;

		// Nodes at ends.
		Node2D *MyNodeA;
		Node2D *MyNodeB;

		bool MyReleaseNodeA[3];
		bool MyReleaseNodeB[3];
		
		double MyReleaseNodeAMagnitude[3]; 
		double MyReleaseNodeBMagnitude[3];

		double MyLocalStiffness[6][6];
		
		vector <BeamLoad2D*> MyVectorBeamLoads;

		static const int DOF = 3;

		void Initialize(int ID);

		void Clone(const Beam2D &Beam);
	public:
		// Constructor
		Beam2D();
		
		Beam2D(int ID,
			   Node2D &NodeA,
			   Node2D &NodeB,
			   Section2D &Section);

		Beam2D(int ID,
			   Node2D &NodeA,
			   Node2D &NodeB,
			   Section2D &Section,
			   bool ReleaseNodeA[3],
			   bool ReleaseNodeB[3]);

		// Destructor
		~Beam2D();
		
		// Assignment Operator.
		Beam2D& operator=(const Beam2D &Beam);
		
		// Copy Constructor.
		Beam2D(const Beam2D &Beam);

		virtual void SetNode(Node2D* NodeAPtr, Node2D* NodeBPtr);
		virtual void SetNodeA(Node2D* NodePtr);
		virtual void SetNodeB(Node2D* NodePtr);

		virtual void SetSection(Section2D* SectionPtr);

		virtual void SetBeamPointerToNode();

		// Return the ID.
		virtual int GetID();

		// Return the Length.
		virtual double GetLength();

		// Return the Angle from the X-Axis. [Unit Circle]
		virtual double GetAngleXAxis();

		// Coordinate Axis
		virtual double GetXCoordinateNodeA();
		virtual double GetYCoordinateNodeA();
		virtual double GetXCoordinateNodeB();
		virtual double GetYCoordinateNodeB();
		
		// Return Node;
		virtual Node2D &GetNodeA();
		virtual Node2D &GetNodeB();
		
		// Returnt the Node Corresponding with the ID.
		virtual Node2D &GetNode(int ID);
		
		
		// Return the ID Index.
		virtual int GetNodeAID();
		virtual int GetNodeBID();
		
		virtual int GetSectionID();

		virtual vector <int> GetDOFIndexNodes();

		// Stiffness Matrix.
		virtual mat GetLocalStiffnessMatrix();
		virtual mat GetLocalGeometricStiffnessMatrix(double AxialLoad);
		virtual mat GetLocalReleaseStiffnessMatrix(const mat StiffnessMatrix, 
												   int NumOfRelease);
		virtual mat GetLocalInternalForces(mat GlobalDisplacements);

		virtual mat GetTransformationMatrix();	
		virtual mat GetTransformationMatrix(double);	
		
		virtual mat GetGlobalStiffnessMatrix();
		virtual mat GetGlobalGeometricStiffnessMatrix();
		virtual mat GetGlobalGeometricStiffnessMatrix(mat LocalLoads);
		virtual mat GetGlobalGeometricStiffnessMatrix(double AxialLoad);

		virtual mat GetPermutationMatrix();
		virtual mat GetReleaseElasticStiffnessMatrix();
		virtual mat GetReleaseForceVector(mat TangentStiffnessMatrix, 
										  mat FreeDisplacements);
	
		virtual void Print(bool ShowMatrix = false); 
		
		// Check to see if the DOF release.
		virtual bool IsAnyDofRelease();

		// Return the Release.
		virtual int GetNumberOfRelease();
		
		// Return the Constrain and Release List.
		virtual vector <int> GetConstraintList();
		virtual vector <int> GetReleaseList();

		bool IsTranslationXRelease(int ID);
		bool IsTranslationXReleaseNodeA();
		bool IsTranslationXReleaseNodeB();

		bool IsTranslationYRelease(int ID);
		bool IsTranslationYReleaseNodeA();
		bool IsTranslationYReleaseNodeB();

		bool IsRotationZRelease(int ID);
		bool IsRotationZReleaseNodeA();
		bool IsRotationZReleaseNodeB();

		// Set the Releases.
		void SetNodeARelease(bool ParallelRelease, 
							 bool TransverseRelease, 
							 bool MomentRelease, 
							 double ParallelMagnitude = 0.0, 
							 double TransverseMagnitude = 0.0, 
							 double MomentMagnitude = 0.0);

		void SetNodeBRelease(bool ParallelRelease, 
							 bool TransverseRelease, 
							 bool MomentRelease, 
							 double ParallelMagnitude = 0.0, 
							 double TransverseMagnitude = 0.0, 
							 double MomentMagnitude = 0.0);

		void SetNodeAReleaseMoment(bool MomentRelease,
								   double MomentMagnitude = 0.0);

		void SetNodeBReleaseMoment(bool MomentRelease,
									double MomentMagnitude = 0.0);

		virtual mat GetMassVector();
		virtual double GetNodeAMass();
		virtual double GetNodeBMass();

		virtual void AddLoad(BeamLoad2D* Load);
		virtual void ClearLoads();

		// *** FORCE VECTOR ***
		// Return the Beam Force Vector.
		virtual mat GetTotalLocalForceVector();
		virtual mat GetBeamLocalForceVector();
		virtual mat GetNodeLocalForceVector();

		virtual mat GetBeamGlobalForceVector();

		virtual double GetPlasticMomentCapacityNodeA(double InternalMoment);
		virtual double GetPlasticMomentCapacityNodeB(double InternalMoment);

		virtual double GetSelfWeightFactor();
};

#endif