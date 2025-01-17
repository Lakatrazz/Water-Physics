using System.Collections;
using System.Collections.Generic;
using System;

using UnityEngine;
using Unity.Plastic.Antlr3.Runtime.Tree;
using System.Text;

namespace WaterPhysics {
    [RequireComponent(typeof(Rigidbody))]
    [DisallowMultipleComponent]
    public class BuoyantBody : MonoBehaviour {
        public sealed class BuoyantCollider : MonoBehaviour {
            public new Collider collider;
            public BuoyantBody attachedBody;

            public Vector3 voxelSize;
            public Vector3[] voxels;
            
            public float percentSubmerged = 0f;

            public void Initiate(Collider collider)
            {
                this.collider = collider;
                var rigidbody = collider.attachedRigidbody;

                if (rigidbody.TryGetComponent(out BuoyantBody buoyant)) {
                    attachedBody = buoyant;
                }
                else {
                    attachedBody = rigidbody.gameObject.AddComponent<BuoyantBody>();
                }
            }

            public void OnFixedUpdate() {
                if (voxels != null && voxels.Length > 0) {
                    var water = attachedBody.water;
                    var rigidbody = attachedBody.rigidbody;

                    Vector3 forceAtSingleVoxel = attachedBody.GetBuoyantForce() / voxels.Length;
                    Bounds bounds = collider.bounds;
                    float voxelHeight = bounds.size.y * attachedBody.normalizedVoxelSize;

                    for (int i = 0; i < voxels.Length; i++)
                    {
                        Vector3 worldPoint = transform.TransformPoint(voxels[i]);

                        float waterLevel = water.GetWaterLevel();
                        float deepLevel = waterLevel - worldPoint.y + (voxelHeight * 0.5f); // How deep is the voxel                    
                        float submergedFactor = Mathf.Clamp01(deepLevel / voxelHeight); // 0 - voxel is fully out of the water, 1 - voxel is fully submerged
                        percentSubmerged += submergedFactor;

                        Vector3 surfaceNormal = water.GetSurfaceNormal();
                        Quaternion surfaceRotation = Quaternion.FromToRotation(water.transform.up, surfaceNormal);
                        surfaceRotation = Quaternion.Slerp(surfaceRotation, Quaternion.identity, submergedFactor);

                        Vector3 finalVoxelForce = surfaceRotation * (forceAtSingleVoxel * submergedFactor);
                        Vector3 finalVoxelTorque = Vector3.Cross(worldPoint - rigidbody.worldCenterOfMass, finalVoxelForce);

                        rigidbody.AddForce(finalVoxelForce, ForceMode.Acceleration);
                        rigidbody.AddTorque(finalVoxelTorque, ForceMode.Acceleration);
                    }

                    percentSubmerged /= voxels.Length; // 0 - object is fully out of the water, 1 - object is fully submerged
                }
            }

            public void UpdateVoxels()
            {
                if (voxels == null)
                    voxels = CutIntoVoxels();
            }

            public void OnDrawGizmos()
            {
                if (voxels != null)
                {
                    for (int i = 0; i < voxels.Length; i++)
                    {
                        Gizmos.color = Color.magenta - new Color(0f, 0f, 0f, 0.75f);
                        Gizmos.DrawCube(transform.TransformPoint(voxels[i]), voxelSize * 0.8f);
                    }
                }
            }

            private Vector3[] CutIntoVoxels()
            {
                Quaternion initialRotation = transform.rotation;
                transform.rotation = Quaternion.identity;

                Bounds bounds = collider.bounds;
                var normalizedVoxelSize = attachedBody.normalizedVoxelSize;

                voxelSize.x = bounds.size.x * normalizedVoxelSize;
                voxelSize.y = bounds.size.y * normalizedVoxelSize;
                voxelSize.z = bounds.size.z * normalizedVoxelSize;
                int voxelsCountForEachAxis = Mathf.RoundToInt(1f / normalizedVoxelSize);
                List<Vector3> voxels = new(voxelsCountForEachAxis * voxelsCountForEachAxis * voxelsCountForEachAxis);

                for (int i = 0; i < voxelsCountForEachAxis; i++)
                {
                    for (int j = 0; j < voxelsCountForEachAxis; j++)
                    {
                        for (int k = 0; k < voxelsCountForEachAxis; k++)
                        {
                            float pX = bounds.min.x + voxelSize.x * (0.5f + i);
                            float pY = bounds.min.y + voxelSize.y * (0.5f + j);
                            float pZ = bounds.min.z + voxelSize.z * (0.5f + k);

                            Vector3 point = new(pX, pY, pZ);
                            if (IsPointInsideCollider(point, collider, ref bounds))
                            {
                                voxels.Add(transform.InverseTransformPoint(point));
                            }
                        }
                    }
                }

                transform.rotation = initialRotation;

                return voxels.ToArray();
            }

            private bool IsPointInsideCollider(Vector3 point, Collider collider, ref Bounds colliderBounds)
            {
                float rayLength = colliderBounds.size.magnitude;
                Ray ray = new Ray(point, collider.transform.position - point);
                RaycastHit hit;

                if (Physics.Raycast(ray, out hit, rayLength))
                {
                    if (hit.collider == collider)
                    {
                        return false;
                    }
                }

                return true;
            }
        }

        [Header("Divisions")]
        [Range(0f, 1f)]
        [Tooltip("The scale normalized size of the split voxels used to apply bouyant torque and force.")]
        public float normalizedVoxelSize = 0.5f;

        [Header("Physics")]
        [Min(1e-5f)] 
        [Tooltip("The amount of bouyancy applied to the body.")]
        public float buoyancy = 1.2f;

        [Tooltip("The fluid drag of the body.")]
        public float drag = 1f;

        [Tooltip("The fluid angular drag of the body.")]
        public float angularDrag = 1f;

        [NonSerialized]
        public List<BuoyantCollider> colliders = new List<BuoyantCollider>();

        [NonSerialized]
        public WaterVolume water;

        [NonSerialized]
        public new Rigidbody rigidbody;

        [NonSerialized]
        public float percentSubmerged;

        private bool inWaterThisFrame;
        private int dullWaterFrameCount;

        protected virtual void Awake() {
            rigidbody = GetComponent<Rigidbody>();

            var colliders = GetComponentsInChildren<Collider>(true);

            foreach (var collider in colliders) {
                if (collider.attachedRigidbody == rigidbody) {
                    var buoyantCollider = collider.gameObject.AddComponent<BuoyantCollider>();
                    buoyantCollider.Initiate(collider);
                    this.colliders.Add(buoyantCollider);
                }
            }
        }

        protected virtual void OnTriggerEnter(Collider other) {
            if (water == null && other.GetComponent<WaterVolume>()) {
                water = other.GetComponent<WaterVolume>();

                foreach (var collider in colliders)
                    collider.UpdateVoxels();
            }
        }

        protected virtual void OnTriggerStay(Collider other) {
            if (water && water.gameObject == other.gameObject) {
                inWaterThisFrame = true;
            }
        }

        protected virtual void FixedUpdate() {
            if (!inWaterThisFrame)
                dullWaterFrameCount++;
            else
                dullWaterFrameCount = 0;

            if (dullWaterFrameCount > 10)
                water = null;
            else if (water) {
                foreach (var collider in colliders)
                    collider.OnFixedUpdate();

                percentSubmerged = GetPercentSubmerged();
                float drag = this.drag * water.drag;
                float angularDrag = this.angularDrag * water.angularDrag;

                ApplyDrag(drag * percentSubmerged, angularDrag * percentSubmerged);
            }

            inWaterThisFrame = false;
        }

        protected virtual float GetPercentSubmerged() {
            float perc = 0f;

            foreach (var collider in colliders)
                perc += collider.percentSubmerged;

            if (colliders.Count > 0)
                perc /= (float)colliders.Count;

            return Mathf.Clamp01(perc);
        }

        protected virtual void ApplyDrag(float drag, float angularDrag) {
            float deltaTime = Time.deltaTime;
            rigidbody.velocity *= (1 - deltaTime * drag);
            rigidbody.angularVelocity *= Mathf.Clamp01(1f - angularDrag * deltaTime);
        }

        public virtual Vector3 GetBuoyantForce()
        {
            if (water)
                return buoyancy / (float)colliders.Count * water.density * -Physics.gravity;
            else
                return Vector3.zero;
        }
    }
}