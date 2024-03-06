#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

#include "transform.h"
#include <math.h>

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
void Interpolator::Interpolate(Motion* pInputMotion, Motion** pOutputMotion, int N)
{
    //Allocate new motion
    *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton());

    //Perform the interpolation
    if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
        LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
    else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
        LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
    else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
        BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
    else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
        BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
    else
    {
        printf("Error: unknown interpolation / angle representation type.\n");
        exit(1);
    }
}

void Interpolator::LinearInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
                interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

//Converts a rotation matrix to an euler angle (roll, pitch, yaw)
void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
    double cy = sqrt(R[0] * R[0] + R[3] * R[3]);

    if (cy > 16 * DBL_EPSILON)
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

    for (int i = 0; i < 3; i++)
        angles[i] *= 180 / M_PI;
}

void multiplyMatrices(const double A[9], const double B[9], double result[9]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i * 3 + j] = 0;
            for (int k = 0; k < 3; ++k) {
                result[i * 3 + j] += A[i * 3 + k] * B[k * 3 + j];
            }
        }
    }
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    double sx = sin(angles[0] * M_PI / 180);
    double cx = cos(angles[0] * M_PI / 180);
    double sy = sin(angles[1] * M_PI / 180);
    double cy = cos(angles[1] * M_PI / 180);
    double sz = sin(angles[2] * M_PI / 180);
    double cz = cos(angles[2] * M_PI / 180);

    double zMatrix[9] = { cz, -sz, 0,
                            sz, cz, 0,
                            0, 0, 1 };
    double yMatrix[9] = { cy, 0, sy,
                            0, 1, 0,
                            -sy, 0, cy };
    double xMatrix[9] = { 1, 0, 0,
                            0, cx, -sx,
                            0, sx, cx };

    double tempMatrix[9];
    multiplyMatrices(zMatrix, yMatrix, tempMatrix);
    multiplyMatrices(tempMatrix, xMatrix, R);
}

vector calculateControlHelperEuler(vector q1, vector q2, vector q3, bool midpoint, double thirdTimeStep) {
    vector doubleStep = q1 + (q2 - q1) * 2.0;
    if (midpoint) {
        vector midpointVec = doubleStep + (q3 - doubleStep) * 0.5;
        return q2 + (midpointVec - q1) * thirdTimeStep;
    }
    return q3 + (doubleStep - q3) * thirdTimeStep;
}

void Interpolator::BezierInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    // Pre-doing division for later
    double thirdTimeStep = (1.0 / 3.0);

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position w/ Bezier
            vector q1 = startPosture->root_pos;
            vector q2 = endPosture->root_pos;
            vector a;
            vector b;

            // Find control point a
                // Special case if this is a1 = Slerp(q1, Slerp(q3, q2, 2.0), (1.0 / 3)
            if (startKeyframe == 0) {
                Posture* postureNplus1 = pInputMotion->GetPosture(endKeyframe + N + 1);
                vector q3 = postureNplus1->root_pos;
                a = calculateControlHelperEuler(q3, q2, q1, false, thirdTimeStep);
            }
            else {
                Posture* postureNminus1 = pInputMotion->GetPosture(startKeyframe - N - 1);
                vector qnminus1 = postureNminus1->root_pos;
                a = calculateControlHelperEuler(qnminus1, q1, q2, true, thirdTimeStep);
            }

            // Find control point b
            // Special case if this is bn = Slerp(qn, Slerp(qn-2, qn-1, 2.0), (1.0 / 3)
            if (endKeyframe + N + 1 > inputLength) {
                Posture* postureNminus1 = pInputMotion->GetPosture(startKeyframe - N - 1);
                vector qnminus1 = postureNminus1->root_pos;
                b = calculateControlHelperEuler(qnminus1, q1, q2, false, thirdTimeStep);
            }
            else {
                Posture* postureNplus1 = pInputMotion->GetPosture(endKeyframe + N + 1);
                vector qnplus1 = postureNplus1->root_pos;
                b = calculateControlHelperEuler(q1, q2, qnplus1, true, -thirdTimeStep);
            }

            interpolatedPosture.root_pos = DeCasteljauEuler(t, q1, a, b, q2);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
                q1 = startPosture->bone_rotation[bone];
                q2 = endPosture->bone_rotation[bone];

                // Find control point a
                // Special case if this is a1 = Slerp(q1, Slerp(q3, q2, 2.0), (1.0 / 3)
                if (startKeyframe == 0) {
                    Posture* postureNplus1 = pInputMotion->GetPosture(endKeyframe + N + 1);
                    vector q3 = postureNplus1->bone_rotation[bone];
                    a = calculateControlHelperEuler(q3, q2, q1, false, thirdTimeStep);
                }
                else {
                    Posture* postureNminus1 = pInputMotion->GetPosture(startKeyframe - N - 1);
                    vector qnminus1 = postureNminus1->bone_rotation[bone];
                    a = calculateControlHelperEuler(qnminus1, q1, q2, true, thirdTimeStep);
                }

                // Find control point b
                // Special case if this is bn = Slerp(qn, Slerp(qn-2, qn-1, 2.0), (1.0 / 3)
                if (endKeyframe + N + 1 > inputLength) {
                    Posture* postureNminus1 = pInputMotion->GetPosture(startKeyframe - N - 1);
                    vector qnminus1 = postureNminus1->bone_rotation[bone];
                    b = calculateControlHelperEuler(qnminus1, q1, q2, false, thirdTimeStep);
                }
                else {
                    Posture* postureNplus1 = pInputMotion->GetPosture(endKeyframe + N + 1);
                    vector qnplus1 = postureNplus1->bone_rotation[bone];
                    b = calculateControlHelperEuler(q1, q2, qnplus1, true, -thirdTimeStep);
                }

                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, q1, a, b, q2);
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
                Quaternion<double> startQuaternion;
                Quaternion<double> endQuaternion;

                // Get quaternions from euler
                vector startBoneRotationEuler = startPosture->bone_rotation[bone];
                double startEuler[3] = { startBoneRotationEuler.x(), startBoneRotationEuler.y(), startBoneRotationEuler.z() };
                Euler2Quaternion(startEuler, startQuaternion);
                vector endBoneRotationEuler = endPosture->bone_rotation[bone];
                double endEuler[3] = { endBoneRotationEuler.x(), endBoneRotationEuler.y(), endBoneRotationEuler.z() };
                Euler2Quaternion(endEuler, endQuaternion);
                Quaternion<double> interpolatedQuaternion = Slerp(t, startQuaternion, endQuaternion);

                // Convert back to euler angles
                double interpolatedEuler[3];
                Quaternion2Euler(interpolatedQuaternion, interpolatedEuler);
                interpolatedPosture.bone_rotation[bone] = interpolatedEuler;
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

Quaternion<double> Interpolator::calculateControlHelperQuaternion(Quaternion<double> q1, Quaternion<double> q2, Quaternion<double> q3, bool midpoint, double thirdTimeStep) {
    Quaternion<double> doubleStep = Double(q1, q2);
    if (midpoint) {
        Quaternion<double> midpointQuaternion = Slerp(0.5, doubleStep, q3);
        return Slerp(thirdTimeStep, q2, midpointQuaternion);
    }
    return Slerp(thirdTimeStep, q3, doubleStep);
}

void Interpolator::BezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    // Pre-doing division for later
    double thirdTimeStep = (1.0 / 3.0);

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position w/ Bezier
            vector q1 = startPosture->root_pos;
            vector q2 = endPosture->root_pos;
            vector a;
            vector b;

            // Find control point a
                // Special case if this is a1 = Slerp(q1, Slerp(q3, q2, 2.0), (1.0 / 3)
            if (startKeyframe == 0) {
                Posture* postureNplus1 = pInputMotion->GetPosture(endKeyframe + N + 1);
                vector q3 = postureNplus1->root_pos;
                a = calculateControlHelperEuler(q3, q2, q1, false, thirdTimeStep);
            }
            else {
                Posture* postureNminus1 = pInputMotion->GetPosture(startKeyframe - N - 1);
                vector qnminus1 = postureNminus1->root_pos;
                a = calculateControlHelperEuler(qnminus1, q1, q2, true, thirdTimeStep);
            }

            // Find control point b
            // Special case if this is bn = Slerp(qn, Slerp(qn-2, qn-1, 2.0), (1.0 / 3)
            if (endKeyframe + N + 1 > inputLength) {
                Posture* postureNminus1 = pInputMotion->GetPosture(startKeyframe - N - 1);
                vector qnminus1 = postureNminus1->root_pos;
                b = calculateControlHelperEuler(qnminus1, q1, q2, false, thirdTimeStep);
            }
            else {
                Posture* postureNplus1 = pInputMotion->GetPosture(endKeyframe + N + 1);
                vector qnplus1 = postureNplus1->root_pos;
                b = calculateControlHelperEuler(q1, q2, qnplus1, true, -thirdTimeStep);
            }

            interpolatedPosture.root_pos = DeCasteljauEuler(t, q1, a, b, q2);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
                Quaternion<double> q1;
                Quaternion<double> q2;
                Quaternion<double> a;
                Quaternion<double> b;

                // Get quaternions from euler
                vector q1BoneRotationEuler = startPosture->bone_rotation[bone];
                double q1Euler[3] = { q1BoneRotationEuler.x(), q1BoneRotationEuler.y(), q1BoneRotationEuler.z() };
                Euler2Quaternion(q1Euler, q1);
                vector q2BoneRotationEuler = endPosture->bone_rotation[bone];
                double q2Euler[3] = { q2BoneRotationEuler.x(), q2BoneRotationEuler.y(), q2BoneRotationEuler.z() };
                Euler2Quaternion(q2Euler, q2);
                // Slerp between start and end rotation quaternion
                // Find control point a
                // Special case if this is a1 = Slerp(q1, Slerp(q3, q2, 2.0), (1.0 / 3)
                if (startKeyframe == 0) {
                    Posture* postureNplus1 = pInputMotion->GetPosture(endKeyframe + N + 1);
                    vector q3BoneRotationEuler = postureNplus1->bone_rotation[bone];
                    double q3Euler[3] = { q3BoneRotationEuler.x(), q3BoneRotationEuler.y(), q3BoneRotationEuler.z() };
                    Quaternion<double> q3;
                    Euler2Quaternion(q3Euler, q3);
                    a = calculateControlHelperQuaternion(q3, q2, q1, false, thirdTimeStep);
                }
                else {
                    Posture* postureNminus1 = pInputMotion->GetPosture(startKeyframe - N - 1);
                    vector qnminus1BoneRotationEuler = postureNminus1->bone_rotation[bone];
                    double eulerNminus1[3] = { qnminus1BoneRotationEuler.x(), qnminus1BoneRotationEuler.y(), qnminus1BoneRotationEuler.z() };
                    Quaternion<double> qnminus1;
                    Euler2Quaternion(eulerNminus1, qnminus1);
                    a = calculateControlHelperQuaternion(qnminus1, q1, q2, true, thirdTimeStep);
                }

                // Find control point b
                // Special case if this is bn = Slerp(qn, Slerp(qn-2, qn-1, 2.0), (1.0 / 3)
                if (endKeyframe + N + 1 > inputLength) {
                    Posture* postureNminus1 = pInputMotion->GetPosture(startKeyframe - N - 1);
                    vector qnminus1BoneRotationEuler = postureNminus1->bone_rotation[bone];
                    double eulerNminus1[3] = { qnminus1BoneRotationEuler.x(), qnminus1BoneRotationEuler.y(), qnminus1BoneRotationEuler.z() };
                    Quaternion<double> qnminus1;
                    Euler2Quaternion(eulerNminus1, qnminus1);
                    b = calculateControlHelperQuaternion(qnminus1, q1, q2, false, thirdTimeStep);
                }
                else {
                    Posture* postureNplus1 = pInputMotion->GetPosture(endKeyframe + N + 1);
                    vector qnplus1BoneRotationEuler = postureNplus1->bone_rotation[bone];
                    double eulerNplus1[3] = { qnplus1BoneRotationEuler.x(), qnplus1BoneRotationEuler.y(), qnplus1BoneRotationEuler.z() };
                    Quaternion<double> qnplus1;
                    Euler2Quaternion(eulerNplus1, qnplus1);
                    b = calculateControlHelperQuaternion(q1, q2, qnplus1, true, -thirdTimeStep);
                }

                Quaternion<double> interpolatedQuaternion = DeCasteljauQuaternion(t, q1, a, b, q2);

                // Convert back to euler angles
                double interpolatedEuler[3];
                Quaternion2Euler(interpolatedQuaternion, interpolatedEuler);
                interpolatedPosture.bone_rotation[bone] = interpolatedEuler;
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double>& q)
{
    double rotationMatrix[9];
    // First convert to 3x3 matrix, then to quaternion
    Euler2Rotation(angles, rotationMatrix);
    q = Quaternion<double>::Matrix2Quaternion(rotationMatrix);
}

void Interpolator::Quaternion2Euler(Quaternion<double>& q, double angles[3])
{
    double rotationMatrix[9];
    // First convert to 3x3 matrix, then to euler
    q.Quaternion2Matrix(rotationMatrix);
    Rotation2Euler(rotationMatrix, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double>& qStart, Quaternion<double>& qEnd_)
{
    Quaternion<double> result;
    double qStartDotqEnd = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();

    // Quaternions are more than halfway around unit quaternion sphere, negate to go the opposite direction which is shorter
    // q = -q so we know we can do this
    if (qStartDotqEnd < 0.0) {
        qStartDotqEnd = -qStartDotqEnd;
        qEnd_.Set(-qEnd_.Gets(), -qEnd_.Getx(), -qEnd_.Gety(), -qEnd_.Getz());
    }

    double theta = acos(qStartDotqEnd);
    // The quaternions were too close together so theta was too small and a division by 0 happened, must do linear interpolation so just set it to start
    if (theta == 0.0) {
        result = qStart;
    }
    else {
        result = (sin((1.0 - t) * theta) / sin(theta)) * qStart + (sin(t * theta) / sin(theta)) * qEnd_;
    }
    return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
    Quaternion<double> result;
    double pDotq = p.Gets() * q.Gets() + p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz();
    result = 2 * pDotq * q - p;
    return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
    vector result;

    vector q0 = p0 + (p1 - p0) * t;
    vector q1 = p1 + (p2 - p1) * t;
    vector q2 = p2 + (p3 - p2) * t;
    vector r0 = q0 + (q0 - q1) * t;
    vector r1 = q1 + (q2 - q1) * t;
    result = r0 + (r1 - r0) * t;

    return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
    Quaternion<double> result;

    Quaternion<double> q0 = Slerp(t, p0, p1);
    Quaternion<double> q1 = Slerp(t, p1, p2);
    Quaternion<double> q2 = Slerp(t, p2, p3);
    Quaternion<double> r0 = Slerp(t, q0, q1);
    Quaternion<double> r1 = Slerp(t, q1, q2);
    result = Slerp(t, r0, r1);
    return result;
}
