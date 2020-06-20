#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include "transform.h"  // For use of rotation matrices in Euler2Rotation
#include "performanceCounter.h" // To find time taken by each interpolation

PerformanceCounter pc;
float LEtimeElapsed = 0.0, LQtimeElapsed = 0.0, BEtimeElapsed = 0.0, BQtimeElapsed = 0.0;
char * dtext = NULL;
char s[40];

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == SLERPQUATERNION))
    LinearInterpolationSlerpQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == SLERPLERPQUATERNION))
    LinearInterpolationSlerpLerpQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == SLERPQUATERNION))
    BezierInterpolationSlerpQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == SLERPLERPQUATERNION))
    BezierInterpolationSlerpLerpQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
  double rotX[4][4], rotY[4][4], rotZ[4][4];  // rotation matrices
  rotationX(rotX,angles[0]);  // rotation about X axis, function also converts degree to rad
  rotationY(rotY,angles[1]);  // rotation about Y axis, function also converts degree to rad
  rotationZ(rotZ,angles[2]);  // rotation about Z axis, function also converts degree to rad
  double temp[4][4], r[4][4];
  matrix_mult(rotZ,rotY,temp);
  matrix_mult(temp,rotX,r);

  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      R[j + 3 * i] = r[i][j];
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) // we convert via intermediate rotation matrix
{
  // students should implement this
  double R[9];  // rotation matrix
  Euler2Rotation(angles,R); // To convert Euler angles in XYZ Euler angle order to rotation matrix
  q = Quaternion<double>::Matrix2Quaternion(R); // To convert matrix to quaternion - more like Rotation2Quaternion
  q.Normalize();  // Since rotations are unit quaternions
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) // we convert via intermediate rotation matrix
{
  // students should implement this
  double R[9];  // rotation matrix
  q.Quaternion2Matrix(R); // To transforms the quaternion to the corresponding rotation matrix - more like Quaternion2Rotation
  Rotation2Euler(R,angles); // To convert rotation matrix to XYZ Euler angle order Euler angles
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

  pc.StartCounter();

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  pc.StopCounter();
  LEtimeElapsed = pc.GetElapsedTime();
  std::cout << "Linear Interpolation Euler - "<< LEtimeElapsed << std::endl;

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

// Implemented following Rick Parent's book part 3.3 Interpolation of orientations
void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  vector p0, p1, p2, p3; // the keyframe points
  vector a1,b2; // control points

  pc.StartCounter();

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;
    int prevKeyframe = startKeyframe - (N + 1);
    int nextKeyframe = endKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
    Posture * prevPosture;
    Posture * nextPosture;

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);
    // get previous and next keyframe
    if (prevKeyframe >= 0)
      prevPosture = pInputMotion->GetPosture(prevKeyframe);
    if (nextKeyframe < inputLength)
      nextPosture = pInputMotion->GetPosture(nextKeyframe);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      p1 = startPosture->root_pos;
      p2 = endPosture->root_pos;

      vector a2; // intermediate needed to compute b2

      if (startKeyframe == 0) // If there's no previous keyframe
      {
        p3 = nextPosture->root_pos;

        // computing a1 - The first control point is constructed as the vector from the third interpolated point to the second point
        // is added to the second point.
        a1 = (p2 - p3) + p2;
        a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        // computing b2 - First we need to compute a2 and then we compute b2 by taking a2 to p2 and adding it to p2.
        a2 = (((p2 - p1) + p2) + p3) * 0.5;
        a2 = p2 + (a2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        b2 = (p2 - a2) + p2;
      }
      else if (nextKeyframe>inputLength)  // If there's no next keyframe after the endKeyframe
      {
        p0 = prevPosture->root_pos;

        // computing a1 - To calculate the control point following any particular point p1, take the vector defined by p0 to p1 and add it to p1.
        // Now, take this point and find the average of it and p2.
        a1 = (((p1 - p0) + p1) + p2) * 0.5;
        a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        // computing b2 - The last control point is constructed as the vector from the first interpolated point to the second point
        // is added to the second point.
        b2 = (p1 - p0) + p1;
        b2 = p2 + (b2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (bn = pn + k(bn - pn) where k = 1/3)
      }
      else  // For all intermediate frames while we have both previous and next keyframes
      {
        p0 = prevPosture->root_pos;
        p3 = nextPosture->root_pos;

        // computing a1 - To calculate the control point following any particular point p1, take the vector defined by p0 to p1 and add it to p1.
        // Now, take this point and find the average of it and p2.
        a1 = (((p1 - p0) + p1) + p2) * 0.5;
        a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        // computing b2 - First we need to compute a2 and then we compute b2 by taking a2 to p2 and adding it to p2.
        a2 = (((p2 - p1) + p2) + p3) * 0.5;
        a2 = p2 + (a2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        b2 = (p2 - a2) + p2;
      }
      interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        p1 = startPosture->bone_rotation[bone];
        p2 = endPosture->bone_rotation[bone];

        if (startKeyframe == 0) // If there's no previous keyframe
        {
          p3 = nextPosture->bone_rotation[bone];

          // computing a1 - The first control point is constructed as the vector from the third interpolated point to the second point
          // is added to the second point.
          a1 = (p2 - p3) + p2;
          a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
          // computing b2 - First we need to compute a2 and then we compute b2 by taking a2 to p2 and adding it to p2.
          a2 = (((p2 - p1) + p2) + p3) * 0.5;
          a2 = p2 + (a2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
          b2 = (p2 - a2) + p2;
        }
        else if (nextKeyframe>inputLength)  // If there's no next keyframe after the endKeyframe
        {
          p0 = prevPosture->bone_rotation[bone];

          // computing a1 - To calculate the control point following any particular point p1, take the vector defined by p0 to p1 and add it to p1.
          // Now, take this point and find the average of it and p2.
          a1 = (((p1 - p0) + p1) + p2) * 0.5;
          a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
          // computing b2 - The last control point is constructed as the vector from the first interpolated point to the second point
          // is added to the second point.
          b2 = (p1 - p0) + p1;
          b2 = p2 + (b2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (bn = pn + k(bn - pn) where k = 1/3)
        }
        else  // For all intermediate frames while we have both previous and next keyframes
        {
          p0 = prevPosture->bone_rotation[bone];
          p3 = nextPosture->bone_rotation[bone];

          // computing a1 - To calculate the control point following any particular point p1, take the vector defined by p0 to p1 and add it to p1.
          // Now, take this point and find the average of it and p2.
          a1 = (((p1 - p0) + p1) + p2) * 0.5;
          a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
          // computing b2 - First we need to compute a2 and then we compute b2 by taking a2 to p2 and adding it to p2.
          a2 = (((p2 - p1) + p2) + p3) * 0.5;
          a2 = p2 + (a2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
          b2 = (p2 - a2) + p2;
        }
        interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1, a1, b2, p2);
      }
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  pc.StopCounter();
  BEtimeElapsed = pc.GetElapsedTime();
  std::cout << "Bezier Interpolation Euler - "<< BEtimeElapsed << std::endl;

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationSlerpQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

  pc.StartCounter();

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t; // Linear Quaternion should use Linear Euler for the root (as per Assignment Requirements).

      // interpolate bone rotations
      Quaternion<double> start, end, boneSlerp;
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        // first converting startPosture and endPosture individual bone's rotation from Euler angles to quaternion
        Euler2Quaternion(startPosture->bone_rotation[bone].p, start); // p has that bone's pose in the given start posture of skeleton (see line 486 in skeleton.cpp)
        Euler2Quaternion(endPosture->bone_rotation[bone].p, end); // p has that bone's pose in the given end posture of skeleton
        boneSlerp = Slerp(t,start,end); // Slerping the bone 
        Quaternion2Euler(boneSlerp, interpolatedPosture.bone_rotation[bone].p); // converting from quaternion back to Euler angles
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  pc.StopCounter();
  LQtimeElapsed = pc.GetElapsedTime();
  std::cout << "Linear Interpolation Slerp Quaternion - "<< LQtimeElapsed << std::endl;

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

// using SLERP for all angles except θ -> 0 where I used LERP
void Interpolator::LinearInterpolationSlerpLerpQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

  pc.StartCounter();

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t; // Linear Quaternion should use Linear Euler for the root (as per Assignment Requirements).

      // interpolate bone rotations
      Quaternion<double> start, end, boneSlerp;
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        // first converting startPosture and endPosture individual bone's rotation from Euler angles to quaternion
        Euler2Quaternion(startPosture->bone_rotation[bone].p, start); // p has that bone's pose in the given start posture of skeleton (see line 486 in skeleton.cpp)
        Euler2Quaternion(endPosture->bone_rotation[bone].p, end); // p has that bone's pose in the given end posture of skeleton
        boneSlerp = SlerpLerp(t,start,end); // Slerping the bone 
        Quaternion2Euler(boneSlerp, interpolatedPosture.bone_rotation[bone].p); // converting from quaternion back to Euler angles
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  pc.StopCounter();
  LQtimeElapsed = pc.GetElapsedTime();
  std::cout << "Linear Interpolation Slerp+Lerp Quaternion - "<< LQtimeElapsed << std::endl;

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationSlerpQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  vector p0, p1, p2, p3; // the keyframe points
  vector a1,b2; // control points
  Quaternion<double> boneq0, boneq1, boneq2, boneq3; // bone keyframe quaternions
  Quaternion<double> bonea1, boneb2;  // bone control quaternions

  pc.StartCounter();

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;
    int prevKeyframe = startKeyframe - (N + 1);
    int nextKeyframe = endKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
    Posture * prevPosture;
    Posture * nextPosture;

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);
    // get previous and next keyframe
    if (prevKeyframe >= 0)
      prevPosture = pInputMotion->GetPosture(prevKeyframe);
    if (nextKeyframe < inputLength)
      nextPosture = pInputMotion->GetPosture(nextKeyframe);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      // Bezier Quaternion should use Bezier Euler for the root (as per Assignment Requirements).
      p1 = startPosture->root_pos;
      p2 = endPosture->root_pos;

      vector a2; // intermediate needed to compute b2

      if (startKeyframe == 0) // If there's no previous keyframe
      {
        p3 = nextPosture->root_pos;

        // computing a1 - The first control point is constructed as the vector from the third interpolated point to the second point
        // is added to the second point.
        a1 = (p2 - p3) + p2;
        a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        // computing b2 - First we need to compute a2 and then we compute b2 by taking a2 to p2 and adding it to p2.
        a2 = (((p2 - p1) + p2) + p3) * 0.5;
        a2 = p2 + (a2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        b2 = (p2 - a2) + p2;
      }
      else if (nextKeyframe>inputLength)  // If there's no next keyframe after the endKeyframe
      {
        p0 = prevPosture->root_pos;

        // computing a1 - To calculate the control point following any particular point p1, take the vector defined by p0 to p1 and add it to p1.
        // Now, take this point and find the average of it and p2.
        a1 = (((p1 - p0) + p1) + p2) * 0.5;
        a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        // computing b2 - The last control point is constructed as the vector from the first interpolated point to the second point
        // is added to the second point.
        b2 = (p1 - p0) + p1;
        b2 = p2 + (b2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (bn = pn + k(bn - pn) where k = 1/3)
      }
      else  // For all intermediate frames while we have both previous and next keyframes
      {
        p0 = prevPosture->root_pos;
        p3 = nextPosture->root_pos;

        // computing a1 - To calculate the control point following any particular point p1, take the vector defined by p0 to p1 and add it to p1.
        // Now, take this point and find the average of it and p2.
        a1 = (((p1 - p0) + p1) + p2) * 0.5;
        a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        // computing b2 - First we need to compute a2 and then we compute b2 by taking a2 to p2 and adding it to p2.
        a2 = (((p2 - p1) + p2) + p3) * 0.5;
        a2 = p2 + (a2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        b2 = (p2 - a2) + p2;
      }
      interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        Euler2Quaternion(startPosture->bone_rotation[bone].p, boneq1);
        Euler2Quaternion(endPosture->bone_rotation[bone].p, boneq2);

        Quaternion<double> bonea2;  // intermediate needed to compute b2
        Quaternion<double> helpCalc;

        if (startKeyframe == 0) // If there's no previous keyframe
        {
          Euler2Quaternion(nextPosture->bone_rotation[bone].p, boneq3);

          // computing a1
          bonea1 = Double(boneq3, boneq2);
          bonea1 = Slerp((1.0 / 3), boneq1, bonea1); // To set the control point at 1/3rd of the original length
          // computing b2 - First we need to compute a2 and then we compute b2.
          helpCalc = Double(boneq1, boneq2);
          bonea2 = Slerp(0.5, helpCalc, boneq3);
          boneb2 = Slerp((-1.0 / 3), boneq2, bonea2); // To set the control point at 1/3rd of the original length
        }
        else if (nextKeyframe>inputLength)  // If there's no next keyframe after the endKeyframe
        {
          Euler2Quaternion(prevPosture->bone_rotation[bone].p, boneq0);

          // computing a1 
          helpCalc = Double(boneq0, boneq1);
          bonea1 = Slerp(0.5, helpCalc, boneq2);
          bonea1 = Slerp((1.0 / 3), boneq1, bonea1); // To set the control point at 1/3rd of the original length 
          // computing b2
          boneb2 = Double(boneq0, boneq1);
          boneb2 = Slerp((1.0 / 3), boneq2, boneb2); // To set the control point at 1/3rd of the original length
        }
        else  // For all intermediate frames while we have both previous and next keyframes
        {
          Euler2Quaternion(prevPosture->bone_rotation[bone].p, boneq0);
          Euler2Quaternion(nextPosture->bone_rotation[bone].p, boneq3);

          // computing a1
          helpCalc = Double(boneq0, boneq1);
          bonea1 = Slerp(0.5, helpCalc, boneq2);
          bonea1 = Slerp((1.0 / 3), boneq1, bonea1); // To set the control point at 1/3rd of the original length
          // computing b2 - First we need to compute a2 and then we compute b2.
          helpCalc = Double(boneq1, boneq2);
          bonea2 = Slerp(0.5, helpCalc, boneq3);
          boneb2 = Slerp((-1.0 / 3), boneq2, bonea2); // To set the control point at 1/3rd of the original length
        }
        helpCalc = DeCasteljauSlerpQuaternion(t, boneq1, bonea1, boneb2, boneq2);
        Quaternion2Euler(helpCalc, interpolatedPosture.bone_rotation[bone].p);
      }
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  pc.StopCounter();
  BQtimeElapsed = pc.GetElapsedTime();
  std::cout << "Bezier Interpolation Slerp Quaternion - "<< BQtimeElapsed << std::endl;

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

// using SLERP for all angles except θ -> 0 where I used LERP
void Interpolator::BezierInterpolationSlerpLerpQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  vector p0, p1, p2, p3; // the keyframe points
  vector a1,b2; // control points
  Quaternion<double> boneq0, boneq1, boneq2, boneq3; // bone keyframe quaternions
  Quaternion<double> bonea1, boneb2;  // bone control quaternions

  pc.StartCounter();

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;
    int prevKeyframe = startKeyframe - (N + 1);
    int nextKeyframe = endKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
    Posture * prevPosture;
    Posture * nextPosture;

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);
    // get previous and next keyframe
    if (prevKeyframe >= 0)
      prevPosture = pInputMotion->GetPosture(prevKeyframe);
    if (nextKeyframe < inputLength)
      nextPosture = pInputMotion->GetPosture(nextKeyframe);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      // Bezier Quaternion should use Bezier Euler for the root (as per Assignment Requirements).
      p1 = startPosture->root_pos;
      p2 = endPosture->root_pos;

      vector a2; // intermediate needed to compute b2

      if (startKeyframe == 0) // If there's no previous keyframe
      {
        p3 = nextPosture->root_pos;

        // computing a1 - The first control point is constructed as the vector from the third interpolated point to the second point
        // is added to the second point.
        a1 = (p2 - p3) + p2;
        a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        // computing b2 - First we need to compute a2 and then we compute b2 by taking a2 to p2 and adding it to p2.
        a2 = (((p2 - p1) + p2) + p3) * 0.5;
        a2 = p2 + (a2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        b2 = (p2 - a2) + p2;
      }
      else if (nextKeyframe>inputLength)  // If there's no next keyframe after the endKeyframe
      {
        p0 = prevPosture->root_pos;

        // computing a1 - To calculate the control point following any particular point p1, take the vector defined by p0 to p1 and add it to p1.
        // Now, take this point and find the average of it and p2.
        a1 = (((p1 - p0) + p1) + p2) * 0.5;
        a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        // computing b2 - The last control point is constructed as the vector from the first interpolated point to the second point
        // is added to the second point.
        b2 = (p1 - p0) + p1;
        b2 = p2 + (b2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (bn = pn + k(bn - pn) where k = 1/3)
      }
      else  // For all intermediate frames while we have both previous and next keyframes
      {
        p0 = prevPosture->root_pos;
        p3 = nextPosture->root_pos;

        // computing a1 - To calculate the control point following any particular point p1, take the vector defined by p0 to p1 and add it to p1.
        // Now, take this point and find the average of it and p2.
        a1 = (((p1 - p0) + p1) + p2) * 0.5;
        a1 = p1 + (a1 - p1) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        // computing b2 - First we need to compute a2 and then we compute b2 by taking a2 to p2 and adding it to p2.
        a2 = (((p2 - p1) + p2) + p3) * 0.5;
        a2 = p2 + (a2 - p2) * (1.0/3); // To set the control point at 1/3rd of the original length (an = pn + k(an - pn) where k = 1/3)
        b2 = (p2 - a2) + p2;
      }
      interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        Euler2Quaternion(startPosture->bone_rotation[bone].p, boneq1);
        Euler2Quaternion(endPosture->bone_rotation[bone].p, boneq2);

        Quaternion<double> bonea2;  // intermediate needed to compute b2
        Quaternion<double> helpCalc;

        if (startKeyframe == 0) // If there's no previous keyframe
        {
          Euler2Quaternion(nextPosture->bone_rotation[bone].p, boneq3);

          // computing a1
          bonea1 = Double(boneq3, boneq2);
          bonea1 = SlerpLerp((1.0 / 3), boneq1, bonea1); // To set the control point at 1/3rd of the original length
          // computing b2 - First we need to compute a2 and then we compute b2.
          helpCalc = Double(boneq1, boneq2);
          bonea2 = SlerpLerp(0.5, helpCalc, boneq3);
          boneb2 = SlerpLerp((-1.0 / 3), boneq2, bonea2); // To set the control point at 1/3rd of the original length
        }
        else if (nextKeyframe>inputLength)  // If there's no next keyframe after the endKeyframe
        {
          Euler2Quaternion(prevPosture->bone_rotation[bone].p, boneq0);

          // computing a1 
          helpCalc = Double(boneq0, boneq1);
          bonea1 = SlerpLerp(0.5, helpCalc, boneq2);
          bonea1 = SlerpLerp((1.0 / 3), boneq1, bonea1); // To set the control point at 1/3rd of the original length 
          // computing b2
          boneb2 = Double(boneq0, boneq1);
          boneb2 = SlerpLerp((1.0 / 3), boneq2, boneb2); // To set the control point at 1/3rd of the original length
        }
        else  // For all intermediate frames while we have both previous and next keyframes
        {
          Euler2Quaternion(prevPosture->bone_rotation[bone].p, boneq0);
          Euler2Quaternion(nextPosture->bone_rotation[bone].p, boneq3);

          // computing a1
          helpCalc = Double(boneq0, boneq1);
          bonea1 = SlerpLerp(0.5, helpCalc, boneq2);
          bonea1 = SlerpLerp((1.0 / 3), boneq1, bonea1); // To set the control point at 1/3rd of the original length
          // computing b2 - First we need to compute a2 and then we compute b2.
          helpCalc = Double(boneq1, boneq2);
          bonea2 = SlerpLerp(0.5, helpCalc, boneq3);
          boneb2 = SlerpLerp((-1.0 / 3), boneq2, bonea2); // To set the control point at 1/3rd of the original length
        }
        helpCalc = DeCasteljauSlerpLerpQuaternion(t, boneq1, bonea1, boneb2, boneq2);
        Quaternion2Euler(helpCalc, interpolatedPosture.bone_rotation[bone].p);
      }
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  pc.StopCounter();
  BQtimeElapsed = pc.GetElapsedTime();
  std::cout << "Bezier Interpolation Slerp+Lerp Quaternion - "<< BQtimeElapsed << std::endl;

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  Quaternion<double> result;
  double angle, costheta, sintheta;
  costheta = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();

  // If the dot product(here costheta) is negative, slerp won't take the shorter path. q and -q are equivalent when the negation is applied to all four components. 
  // We just need to reverse one quaternion.
  if (costheta < 0)
  {
    qEnd_ = (-1.0) * qEnd_; // Multiply quaternion with a scalar; e.g. q1 = alpha * q2;
    costheta = -costheta;
  }

  angle = acos(costheta);

  if (angle == 0.0)
    result = qStart;
  else
  {
    sintheta = sin(angle);
    result = (sin((1-t) * angle) / sintheta) * qStart + (sin(t * angle) / sintheta) * qEnd_;
  }
  return result;
}

// If the inputs are too close for comfort, i.e. if the angle is too small i.e. if dot product is below a threshold we lerp and normalize the result.
// Otherwise we do slerp.
Quaternion<double> Interpolator::SlerpLerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  Quaternion<double> result;
  double angle, costheta, sintheta;
  costheta = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();

  // If the dot product(here costheta) is negative, slerp won't take the shorter path. q and -q are equivalent when the negation is applied to all four components. 
  // We just need to reverse one quaternion.
  if (costheta < 0)
  {
    qEnd_ = (-1.0) * qEnd_; // Multiply quaternion with a scalar; e.g. q1 = alpha * q2;
    costheta = -costheta;
  }

  angle = acos(costheta);
  const double costheta_threshold = 0.9995;

  // If the inputs are too close for comfort, we linearly interpolate and normalize the result.
  if(costheta > costheta_threshold)
  {
    result = (1-t) * qStart + t * qEnd_;
    result.Normalize();
  }
  else if (angle == 0.0)
    result = qStart;
  else
  {
    sintheta = sin(angle);
    result = (sin((1-t) * angle) / sintheta) * qStart + (sin(t * angle) / sintheta) * qEnd_;
  }
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q) // Double(p, q) = 2(p*q)q – p, p*q is a dot product
{
  // students should implement this
  Quaternion<double> result;
  double costheta;
  costheta = p.Gets() * q.Gets() + p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz();
  result = ((2 * costheta) * q) - p;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3) // Following slide 30 of Quaternions and Rotations
{
  // students should implement this
  vector result;
  vector Q0, Q1, Q2, R0, R1;
  Q0 = p0*(1-t) + p1*t;
  Q1 = p1*(1-t) + p2*t;
  Q2 = p2*(1-t) + p3*t;
  R0 = Q0*(1-t) + Q1*t;
  R1 = Q1*(1-t) + Q2*t;
  result = R0*(1-t) + R1*t;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauSlerpQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3) // Following slide 31 of Quaternions and Rotations
{
  // students should implement this
  Quaternion<double> result;
  Quaternion<double> Q0, Q1, Q2, R0, R1;
  Q0 = Slerp(t,p0,p1);
  Q1 = Slerp(t,p1,p2);
  Q2 = Slerp(t,p2,p3);
  R0 = Slerp(t,Q0,Q1);
  R1 = Slerp(t,Q1,Q2);
  result = Slerp(t,R0,R1);
  return result;
}

// using SLERP for all angles except θ -> 0 where I used LERP
Quaternion<double> Interpolator::DeCasteljauSlerpLerpQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3) // Following slide 31 of Quaternions and Rotations
{
  // students should implement this
  Quaternion<double> result;
  Quaternion<double> Q0, Q1, Q2, R0, R1;
  Q0 = SlerpLerp(t,p0,p1);
  Q1 = SlerpLerp(t,p1,p2);
  Q2 = SlerpLerp(t,p2,p3);
  R0 = SlerpLerp(t,Q0,Q1);
  R1 = SlerpLerp(t,Q1,Q2);
  result = SlerpLerp(t,R0,R1);
  return result;
}