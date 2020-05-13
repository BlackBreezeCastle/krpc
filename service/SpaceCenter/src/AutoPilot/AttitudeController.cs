using System;
using System.Diagnostics.CodeAnalysis;
using KRPC.SpaceCenter.ExtensionMethods;
using KRPC.SpaceCenter.Services;
using UnityEngine;

namespace KRPC.SpaceCenter.AutoPilot
{
    /// <summary>
    /// Controller to hold a vessels attitude in a chosen orientation.
    /// </summary>
    [SuppressMessage ("Gendarme.Rules.Maintainability", "AvoidLackOfCohesionOfMethodsRule")]
    [SuppressMessage ("Gendarme.Rules.Smells", "AvoidLargeClassesRule")]
    sealed class AttitudeController
    {
        readonly Services.Vessel vessel;
        public readonly PIDController PitchPID = new PIDController (0);
        public readonly PIDController RollPID = new PIDController (0);
        public readonly PIDController YawPID = new PIDController (0);

        // Target direction
        double targetPitch;
        double targetHeading;
        double targetRoll;
        Vector3d targetDirection;
        QuaternionD targetRotation;
        //ReferenceFrame referenceFrame;
        // Perform control adjustments 10 times per second
        const float timePerUpdate = 0.1f;
        float deltaTime;

        // PID autotuning variables
        Vector3d overshoot;
        Vector3d timeToPeak;
        Vector3d twiceZetaOmega = Vector3d.zero;
        Vector3d omegaSquared = Vector3d.zero;
        QuaternionD directionBiasQuaternion= QuaternionD.identity;
        [SuppressMessage ("Gendarme.Rules.Maintainability", "VariableNamesShouldNotMatchFieldNamesRule")]
        public AttitudeController (Vessel vessel)
        {
            this.vessel = new Services.Vessel (vessel);
            ReferenceFrame = this.vessel.SurfaceReferenceFrame;
            StoppingTime = new Vector3d (0.5, 0.5, 0.5);
            DecelerationTime = new Vector3d (5, 5, 5);
            AttenuationAngle = new Vector3d (1, 1, 1);
            RollThreshold = 360;
            AutoTune = true;
            Overshoot = new Vector3d (0.01, 0.01, 0.01);
            TimeToPeak = new Vector3d (3, 3, 3);
            PlaneMode = false;
            Start ();
        }

        public ReferenceFrame ReferenceFrame { get; set; }
        //{ get { return referenceFrame; } set { referenceFrame = value; UpdateTarget(); } }

        public double TargetPitch {
            get { return targetPitch; }
            set {
                targetPitch = value;
                UpdateTarget ();
            }
        }

        public double TargetHeading {
            get { return targetHeading; }
            set {
                targetHeading = value;
                UpdateTarget ();
            }
        }

        public double TargetRoll {
            get { return targetRoll; }
            set {
                targetRoll = value;
                UpdateTarget ();
            }
        }

        public Vector3d DirectionBias {
            set
            {
                var phr = value;
                directionBiasQuaternion=GeometryExtensions.QuaternionFromPitchHeadingRoll(phr);
            }
            get { return directionBiasQuaternion.PitchHeadingRoll();}
        }

        public bool PlaneMode { get; set; }

        public Vector3d TargetDirection {
            get { return targetDirection; }
            set
            {
                // FIXME: QuaternionD.FromToRotation method not available at runtime
                var direction = ReferenceFrame.Rotation* value;
                direction = vessel.SurfaceReferenceFrame.Rotation.Inverse() * direction;
                var rotation = (QuaternionD)Quaternion.FromToRotation(Vector3d.up, direction);
                var phr = rotation.PitchHeadingRoll();
                TargetPitch = phr.x;
                TargetHeading = phr.y;
            }
        }

        public QuaternionD TargetRotation {
            get { return targetRotation; }
        }

        void UpdateTarget ()
        {
            var phr = new Vector3d (targetPitch, targetHeading, double.IsNaN (targetRoll) ? 0 : targetRoll);
            targetRotation = GeometryExtensions.QuaternionFromPitchHeadingRoll (phr);
            targetRotation= vessel.SurfaceReferenceFrame.Rotation*targetRotation;
            targetRotation = ReferenceFrame.Rotation.Inverse()*targetRotation;
            targetDirection = targetRotation * Vector3.up;
        }

        public Vector3d StoppingTime { get; set; }

        public Vector3d DecelerationTime { get; set; }

        public Vector3d AttenuationAngle { get; set; }

        public double RollThreshold { get; set; }

        public bool AutoTune { get; set; }

        public Vector3d Overshoot {
            get { return overshoot; }
            set {
                overshoot = value;
                UpdatePIDParameters ();
            }
        }

        public Vector3d TimeToPeak {
            get { return timeToPeak; }
            set {
                timeToPeak = value;
                UpdatePIDParameters ();
            }
        }

        void UpdatePIDParameters ()
        {
            for (int i = 0; i < 3; i++) {
                var logOvershoot = Math.Log (overshoot [i]);
                var sqLogOvershoot = logOvershoot * logOvershoot;
                var zeta = Math.Sqrt (sqLogOvershoot / (Math.PI * Math.PI + sqLogOvershoot));
                var omega = Math.PI / (timeToPeak [i] * Math.Sqrt (1.0 - zeta * zeta));
                twiceZetaOmega [i] = 2 * zeta * omega;
                omegaSquared [i] = omega * omega;
            }
        }

        public void Start ()
        {
            PitchPID.Reset (0);
            RollPID.Reset (0);
            YawPID.Reset (0);
        }

        public void Update (PilotAddon.ControlInputs state)
        {
            // Run the controller once every timePerUpdate seconds
            deltaTime += Time.fixedDeltaTime;
            if (deltaTime < timePerUpdate)
                return;
            var internalVessel = vessel.InternalVessel;
            var torque = vessel.AvailableTorqueVectors.Item1;
            var moi = vessel.MomentOfInertiaVector;

            // Compute the input and error for the controllers
            var target = ComputeTargetAngularVelocity (torque, moi);
            var current = ComputeCurrentAngularVelocity ();
            // If roll not set, or not close to target direction, set roll target velocity to 0
            var currentDirection = ReferenceFrame.DirectionFromWorldSpace (internalVessel.ReferenceTransform.up);
            if (double.IsNaN (TargetRoll))
                target.y = 0;

            // Autotune the controllers if enabled
            if (AutoTune)
                DoAutoTune (torque, moi);

            // If vessel is sat on the pad, zero out the integral terms
            if (internalVessel.situation == Vessel.Situations.PRELAUNCH) {
                PitchPID.ClearIntegralTerm ();
                RollPID.ClearIntegralTerm ();
                YawPID.ClearIntegralTerm ();
            }

            // Run per-axis PID controllers
            var output = new Vector3d (
                             PitchPID.Update (target.x, current.x, deltaTime),
                             RollPID.Update (target.y, current.y, deltaTime),
                             YawPID.Update (target.z, current.z, deltaTime));

            state.Pitch = (float)output.x;
            state.Roll = (float)output.y;
            state.Yaw = (float)output.z;

            deltaTime = 0;
        }

        /// <summary>
        /// Compute current angular velocity in pitch,roll,yaw axes
        /// </summary>
        Vector3 ComputeCurrentAngularVelocity ()
        {
            var worldAngularVelocity = vessel.InternalVessel.GetComponent<Rigidbody> ().angularVelocity;
            var localAngularVelocity = ReferenceFrame.AngularVelocityFromWorldSpace (worldAngularVelocity);
            
            // TODO: why does this need to be negative?
            var ret=-vessel.ReferenceFrame.DirectionFromWorldSpace (ReferenceFrame.DirectionToWorldSpace (localAngularVelocity));
            if (PlaneMode)
            {
                ret=ret-TurnAngularVelocity();
            }
            return ret;
        }

        Vector3 TurnAngularVelocity()
        {
            var InternalVessel = vessel.InternalVessel;
            var va = InternalVessel.acceleration;
            var v = InternalVessel.srf_velocity;
            if (va.magnitude<=0||v.magnitude<=0)
            {
                return Vector3d.zero;
            }
            //Debug.Log("va:" + va + "v:" + v);
            va = va - Vector3.Dot(va, v.normalized) * v.normalized;
            var a_mag = va.magnitude / v.magnitude;
            var res = Vector3.Cross(va, v).normalized *(float)a_mag;
            res= vessel.ReferenceFrame.Rotation.Inverse() * res;
            Debug.Log("res"+res+"mag"+res.magnitude);
            return res;
        }
        /// <summary>
        /// Compute target angular velocity in pitch,roll,yaw axes
        /// </summary>
        Vector3 ComputeTargetAngularVelocity(Vector3d torque, Vector3d moi)
        {
            var angles = new Vector3d(0, 0, 0);
            var internalVessel = vessel.InternalVessel;
            var currentRotation = ReferenceFrame.RotationFromWorldSpace(internalVessel.ReferenceTransform.rotation);
            var tmpTargetRotation = TargetRotation;
            var tmpTargetDirection = TargetDirection;

            currentRotation = directionBiasQuaternion * currentRotation;
            var currentDirection = currentRotation * Vector3d.up;

            if (PlaneMode)
            {
                var tmpdirection = internalVessel.GetSrfVelocity();
                if (tmpdirection.magnitude<=0)
                {
                    tmpdirection = ReferenceFrame.Rotation * currentDirection;
                }
                tmpdirection = vessel.SurfaceReferenceFrame.Rotation.Inverse() * tmpdirection;
                var sq =  GeometryExtensions.ToQuaternion(tmpdirection.normalized, TargetRoll);
                sq = ReferenceFrame.Rotation.Inverse() * vessel.SurfaceReferenceFrame.Rotation * sq;
                var right = sq * Vector3d.right;
                var forward = sq * Vector3d.forward;
                var tmpq = QuaternionD.AngleAxis(-(float)TargetPitch,right);
                tmpTargetRotation = tmpq * sq;
                tmpq= QuaternionD.AngleAxis(-(float)TargetHeading, forward);
                tmpTargetRotation = tmpq * tmpTargetRotation;
                tmpTargetDirection = tmpTargetRotation*Vector3d.up;
            }


            QuaternionD rotation = Quaternion.FromToRotation(currentDirection, tmpTargetDirection);
            // Compute angles for the rotation in pitch (x), roll (y), yaw (z) axes
            float angleFloat;
            Vector3 axisFloat;
            // FIXME: QuaternionD.ToAngleAxis method not available at runtime
            ((Quaternion)rotation).ToAngleAxis(out angleFloat, out axisFloat);
            double angle = GeometryExtensions.ClampAngle180(angleFloat);
            Vector3d axis = axisFloat;
            angles = angles + axis * angle;
            if (!double.IsNaN(TargetRoll))
            {
                // Roll angle set => use rotation from currentRotation -> targetRotation
                rotation = rotation.Inverse() * tmpTargetRotation;
                rotation = rotation*currentRotation.Inverse();
                ((Quaternion)rotation).ToAngleAxis(out angleFloat, out axisFloat);
                if (!float.IsInfinity(axisFloat.magnitude))
                {
                    angle = GeometryExtensions.ClampAngle180(angleFloat);
                    axis = axisFloat;
                    angles = angles + axis * angle;
                }
            }
            // else Roll angle not set => use rotation from currentDirection -> targetDirection
            // FIXME: QuaternionD.FromToRotation method not available at runtime
            angles = directionBiasQuaternion * angles;
            angles = vessel.ReferenceFrame.DirectionFromWorldSpace(ReferenceFrame.DirectionToWorldSpace(angles));
            return AnglesToAngularVelocity (angles, torque, moi);
        }

        /// <summary>
        /// Convert a vector of angles to a vector of angular velocities. This
        /// implements the function f(x) from the documentation.
        /// </summary>
        Vector3d AnglesToAngularVelocity (Vector3d angles, Vector3d torque, Vector3d moi)
        {
            var result = Vector3d.zero;
            for (int i = 0; i < 3; i++) {
                var theta = GeometryExtensions.ToRadians (angles [i]);
                var maxAcceleration = torque [i] / moi [i];
                var maxVelocity = maxAcceleration * StoppingTime [i];
                var acceleration = Math.Min (maxAcceleration, maxVelocity / DecelerationTime [i]);
                var velocity = -Math.Sign (angles [i]) * Math.Min (maxVelocity, Math.Sqrt (2.0 * Math.Abs (theta) * acceleration));
                var attenuationAngle = GeometryExtensions.ToRadians (AttenuationAngle [i]);
                var attenuation = 1.0 / (1.0 + Math.Exp (-((Math.Abs (theta) - attenuationAngle) * (6.0 / attenuationAngle))));
                if (double.IsNaN (attenuation))
                    attenuation = 0;
                result [i] = velocity * attenuation;
            }
            return result;
        }

        void DoAutoTune (Vector3d torque, Vector3d moi)
        {
            DoAutoTuneAxis (PitchPID, 0, torque, moi);
            DoAutoTuneAxis (RollPID, 1, torque, moi);
            DoAutoTuneAxis (YawPID, 2, torque, moi);
        }

        void DoAutoTuneAxis (PIDController pid, int index, Vector3d torque, Vector3d moi)
        {
            var accelerationInv = moi [index] / torque [index];
            // Don't tune when the available acceleration is less than 0.001 radian.s^-2
            if (accelerationInv > 1000)
                return;
            var kp = twiceZetaOmega [index] * accelerationInv;
            var ki = omegaSquared [index] * accelerationInv;
            pid.SetParameters (kp, ki, 0, -1, 1);
        }
    }
}
