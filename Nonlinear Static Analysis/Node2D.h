
#pragma once

#ifndef Node2D_H
#define Node2D_H

#include "stdafx.h"
#include "Beam2D.h"
#include "NodeLoad2D.h"
#include "DegreeOfFreedom2D.h"

using namespace std;

// Forward Declaration.
class Beam2D;
class NodeLoad2D;
class DegreeOfFreedom2D;

class Node2D
{
	protected:
		int MyID;							// Nodal ID

		double MyX;							// X Coordinate of Node.
		double MyY;							// Y Coordinate of Node.
		
		double MyRotation;					// Rotation of the Node.

		// Degrees of Freedom
		bool MyDofTranslationX;				// Translation about the X-Axis
		bool MyDofTranslationY;				// Translation about the Y-Axis
		bool MyDofRotationZ;				// Rotation about the Z-Axis
		
		int MyNodeDOFIndexX;
		int MyNodeDOFIndexY;
		int MyNodeDOFIndexZ;

		/// <summary>Number of Beams Connected to this Node</summary>
		vector <Beam2D*> MyVectorBeams;
		vector <NodeLoad2D*> MyVectorNodeLoads;

		// Degree of Freedom Index for the Beams.
		vector <int> MyBeamDOFIndexX;
		vector <int> MyBeamDOFIndexY;
		vector <int> MyBeamDOFIndexZ;
		
		int GetBeamIndex(int BeamID);

		void Initialize(int ID, 
						double X, 
						double Y, 
						bool ReleaseTranslationX, 
						bool ReleaseTranslationY, 
						bool ReleaseRotationZ);

		void Clone(const Node2D &Node);
	public:
		static const int DOF = 3;

		void IncrementTranslationX(double DeltaX);
		void IncrementTranslationY(double DeltaY);
		void IncrementRotationZ(double DeltaZ);

		/// <summary>Default Constructor.</summary>
		Node2D();

		/// <summary>Constructor </summary>
		Node2D(int ID, 
				double X, 
				double Y, 
				bool DofTranslationX, 
				bool DofTranslationY, 
				bool DofRotationZ);
		
		/// <summary>Copy Constructor.</summary>
		Node2D(const Node2D& Node);

		/// <summary> Destructor </summary>
		~Node2D();
		
		Node2D CopyNode(Node2D* Node);

		Node2D& operator=(const Node2D &Node);
		void operator+= (const Node2D &Node);

		// Return Functions
		int GetID();

		double GetX();
		double GetY();
		double GetRotationZ();

		bool GetDofTranslationX();
		bool GetDofTranslationY();
		bool GetDofRotationZ();

		void Print();

		void ClearAll();

		void AddBeam(Beam2D* BeamPtr);
		void ClearBeams(void);

		void AddLoad(NodeLoad2D* LoadPtr);
		void ClearLoads(void);

		// Set the next series of Degrees of Freedom.
		int SetDOFIndex(int NextAvaliableIndex);

		void SetDOF(vector <DegreeOfFreedom2D> &DOF);
		void SetDOF(bool DofTranslationX, 
					bool DofTranslationY, 
					bool DofRotationZ);

		int SetDOFIndexX(int NextAvaliableIndex);
		int SetDOFIndexY(int NextAvaliableIndex);
		int SetDOFIndexZ(int NextAvaliableIndex);
		
		bool AnyReleaseIndexX();
		bool AnyReleaseIndexY();
		bool AnyReleaseIndexZ();
		
		int GetNumOfDofTranslationX();
		int GetNumOfDofTranslationY();
		int GetNumOfDofRotationZ();
		
		// Return the Index base off of the Beam ID.
		int GetBeamDOFIndexX(int BeamID);
		int GetBeamDOFIndexY(int BeamID);
		int GetBeamDOFIndexZ(int BeamID);

		vector <int> GetBeamDOFIndexes(int BeamID);

		vector <int> GetNodeDOFIndexes();

		vector <int> GetNodeRestrainedIndexes();

		mat GetLocalForceVector();

		vector <int> GetDOFIndexX();
		vector <int> GetDOFIndexY();
		vector <int> GetDOFIndexZ();

		void CheckBeamMomentHinging();
};
#endif